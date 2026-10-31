/* Host tests for barcode_peak.h (peak decoder v0.1). No SDK, no hardware.
 *   cc -std=c11 -Wall -Wextra -Werror -O1 -I.. test_barcode_peak.c -lm */
#include <stdio.h>
#include <stdlib.h>
#include "src/barcode/barcode_peak.h"
#include "tests/barcode/bp_captured_fixtures.h"

static int checks, fails;
#define CHECK(c, msg)                                                                              \
    do {                                                                                           \
        ++checks;                                                                                  \
        if (!(c)) {                                                                                \
            ++fails;                                                                               \
            printf("FAIL %s (line %d)\n", msg, __LINE__);                                          \
        }                                                                                          \
    } while (0)

/* ---------- deterministic noise ---------- */
static uint64_t rng = 88172645463325252ull;
static double urand(void) {
    rng ^= rng << 13;
    rng ^= rng >> 7;
    rng ^= rng << 17;
    return (double)(rng >> 11) / 9007199254740992.0;
}
static double nrand(void) {
    double u = urand() + 1e-12, v = urand();
    return sqrt(-2 * log(u)) * cos(6.283185307179586 * v);
}

/* ---------- synthetic label + optics ---------- */
static const char *ALPHA = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. ";
static uint16_t code_of(char c) {
    for (uint16_t p = 0; p < 512; ++p)
        if (bp_character(p) == c)
            return p;
    return 0;
}
typedef struct {
    double a, b;
} Iv;
/* Black intervals of *c* starting at x0, narrow width u, wide = 3u. Optionally reversed. */
static unsigned label(char c, double x0, double u, bool rev, Iv *iv) {
    uint16_t pats[3] = {code_of('*'), code_of(c), code_of('*')};
    int el[29];
    unsigned k = 0;
    for (unsigned ch = 0; ch < 3; ++ch) {
        for (int bit = 8; bit >= 0; --bit)
            el[k++] = (pats[ch] >> bit) & 1;
        if (ch < 2)
            el[k++] = 0;
    }
    if (rev)
        for (unsigned i = 0; i < 14; ++i) {
            int t = el[i];
            el[i] = el[28 - i];
            el[28 - i] = t;
        }
    double x = x0;
    unsigned n = 0;
    for (unsigned i = 0; i < 29; ++i) {
        double w = el[i] ? 3 * u : u;
        if (i % 2 == 0)
            iv[n++] = (Iv){x, x + w};
        x += w;
    }
    return n;
}
/* Footprint: steep saturating core (sigma) + weak linear halo (sigma 4.5 mm, H ADC) that gives the
 * long low tails seen around wide bars on the car. Matches the 2026-10-30 captures by eye. */
typedef struct {
    double sigma, base, K, p, noise, scale, speed, stop_prob, H;
} Optics;
static double adc_at(const Iv *iv, unsigned n, double x, const Optics *o);
static double coverage(const Iv *iv, unsigned n, double x, double s) {
    double c = 0, k = 1.0 / (s * sqrt(2.0));
    for (unsigned i = 0; i < n; ++i)
        c += 0.5 * (erf((iv[i].b - x) * k) - erf((iv[i].a - x) * k));
    return c < 0 ? 0 : c > 1 ? 1 : c;
}
static double adc_at(const Iv *iv, unsigned n, double x, const Optics *o) {
    return o->base + o->K * pow(coverage(iv, n, x, o->sigma), o->p) +
           o->H * coverage(iv, n, x, 4.5);
}
/* Drive the sensor over [0, len] mm; feed the decoder; return decoded codes. */
static unsigned drive(const Iv *iv, unsigned n, double len, const Optics *o, char *codes,
                      unsigned max, BpResult *last, unsigned *groups) {
    BarcodePeak p;
    bp_reset(&p);
    unsigned got = 0;
    double x = 0;
    *groups = 0;
    while (x < len) {
        double adc = adc_at(iv, n, x, o) + o->noise * nrand();
        if (adc < 0)
            adc = 0;
        if (adc > 4095)
            adc = 4095;
        BpResult r;
        if (bp_sample(&p, (uint16_t)(adc + 0.5), x * o->scale, &r)) {
            if (r.nbars >= BP_REPORT_MIN_BARS)
                ++*groups;
            if (r.decoded && got < max)
                codes[got++] = r.code;
            if (last)
                *last = r;
        }
        if (urand() < o->stop_prob)
            for (int k = 0; k < 40; ++k) { /* stutter: 80 ms stop */
                BpResult r2;
                double a2 = adc_at(iv, n, x, o) + o->noise * nrand();
                if (bp_sample(&p, (uint16_t)(a2 < 0 ? 0 : a2 + 0.5), x * o->scale, &r2) &&
                    r2.decoded && got < max)
                    codes[got++] = r2.code;
            }
        x += o->speed * 0.00213 * (0.7 + 0.6 * urand());
    }
    return got;
}
/* Narrow bar 3 mm, footprint sigma: choose K,p so narrow rise ~ target and wide ~1800. */
static void tune(Optics *o, double u, double narrow_rise, double wide_rise) {
    double k = 1.0 / (o->sigma * sqrt(2.0));
    double cn = erf(0.5 * u * k), cw = erf(1.5 * u * k);
    o->p = log(narrow_rise / wide_rise) / log(cn / cw);
    o->K = wide_rise / pow(cw, o->p);
}

/* ---------- A: captured failures ---------- */
static void test_captures(void) {
    for (unsigned i = 0; i < sizeof bp_captures / sizeof bp_captures[0]; ++i) {
        const BpCapture *c = &bp_captures[i];
        BarcodePeak p;
        bp_reset(&p);
        BpResult r, got = {0};
        unsigned decodes = 0, groups = 0;
        for (unsigned k = 0; k < c->n; ++k)
            if (bp_sample(&p, c->adc[k], c->path100[k] / 100.0, &r)) {
                if (r.nbars >= BP_REPORT_MIN_BARS)
                    ++groups;
                if (r.decoded) {
                    ++decodes;
                    got = r;
                }
            }
        /* captures stop 2048 samples in; flush with quiet white beyond the frame */
        double mm = c->path100[c->n - 1] / 100.0;
        for (unsigned k = 0; k < 400; ++k)
            if (bp_sample(&p, 175, mm + 0.1 * k, &r)) {
                if (r.nbars >= BP_REPORT_MIN_BARS)
                    ++groups;
                if (r.decoded) {
                    ++decodes;
                    got = r;
                }
            }
        printf(
            "capture %-16s decoded %c (%s) groups %u bars %u unit %.2f mm narrow-space max %.2f u, wide-space min %.2f u, narrow rise min %.0f\n",
            c->name, decodes ? got.code : '-', got.reverse ? "rev" : "fwd", groups, got.nbars,
            got.unit_mm, got.max_narrow_space, got.min_wide_space, got.min_narrow_rise);
        CHECK(decodes == 1 && got.code == c->expect,
              "captured failure decodes to the known letter exactly once");
        CHECK(groups == 1, "one reported bar group per capture");
        /* the same capture with odometry scale +/-10 %, every 2nd sample, and +/-15 ADC offset */
        const double scales[] = {0.9, 1.1, 1.0, 1.0, 1.0};
        const unsigned steps[] = {1, 1, 2, 1, 1};
        const int offs[] = {0, 0, 0, -15, 15};
        for (unsigned v = 0; v < 5; ++v) {
            BarcodePeak q;
            bp_reset(&q);
            unsigned d2 = 0;
            char c2 = 0;
            for (unsigned k = 0; k < c->n; k += steps[v])
                if (bp_sample(&q, (uint16_t)(c->adc[k] + offs[v]),
                              scales[v] * c->path100[k] / 100.0, &r) &&
                    r.decoded) {
                    ++d2;
                    c2 = r.code;
                }
            for (unsigned k = 0; k < 400; ++k)
                if (bp_sample(&q, (uint16_t)(175 + offs[v]), scales[v] * mm + 0.1 * k, &r) &&
                    r.decoded) {
                    ++d2;
                    c2 = r.code;
                }
            if (!(d2 == 1 && c2 == c->expect))
                printf("  variant %u of %s: decodes %u code %c\n", v, c->name, d2, c2 ? c2 : 0x2d);
            CHECK(d2 == 1 && c2 == c->expect,
                  "capture variant (odometry scale, half sample rate, ADC offset) still decodes");
        }
    }
}

/* ---------- B: synthetic labels, all supported characters, both directions ---------- */
static void test_synthetic(void) {
    unsigned trials = 0, ok = 0, mis = 0;
    for (unsigned li = 0; ALPHA[li]; ++li) {
        char c = ALPHA[li];
        uint16_t pc = code_of(c);
        unsigned wb = 0;
        for (int e = 8; e >= 0; e -= 2)
            wb += (pc >> e) & 1;
        if (wb != 2)
            continue;
        for (unsigned rev = 0; rev < 2; ++rev)
            for (unsigned s = 0; s < 12; ++s) {
                Optics o = {.sigma = 1.5 + 0.6 * urand(),
                            .H = 20 + 40 * urand(),
                            .base = 165 + 20 * urand(),
                            .noise = 1 + 3 * urand(),
                            .scale = 0.92 + 0.16 * urand(),
                            .speed = 30 + 40 * urand(),
                            .stop_prob = s % 3 == 0 ? 0.004 : 0};
                double u = 2.8 + 0.4 * urand();
                tune(&o, u, 25 + 35 * urand(), 1500 + 800 * urand());
                Iv iv[16];
                unsigned n = label(c, 40, u, rev, iv);
                char got[4];
                unsigned groups;
                BpResult last;
                unsigned k = drive(iv, n, 40 + 47 * u + 60, &o, got, 4, &last, &groups);
                ++trials;
                if (k == 1 && got[0] == c)
                    ++ok;
                for (unsigned j = 0; j < k; ++j)
                    if (got[j] != c)
                        ++mis;
                if (!(k == 1 && got[0] == c) && trials - ok <= 3)
                    printf(
                        "  synthetic miss %c rev=%u: k=%u reason=%u bars=%u sigma=%.2f u=%.2f noise=%.1f\n",
                        c, rev, k, last.reason, last.nbars, o.sigma, u, o.noise);
            }
    }
    printf("synthetic: %u/%u decoded, %u misreads\n", ok, trials, mis);
    CHECK(mis == 0, "synthetic labels: no misreads");
    CHECK(ok * 100 >= trials * 99,
          "synthetic labels (realistic blur/noise/speed/stutter): >= 99 % decoded");
}

/* ---------- C: harsh conditions -> may reject, must not misread ---------- */
static void test_harsh(void) {
    unsigned trials = 0, mis = 0, ok = 0;
    for (unsigned t = 0; t < 2000; ++t) {
        char c = "AFZYMW0Q"[t % 8];
        Optics o = {.sigma = 1.4 + 1.2 * urand(),
                    .H = 90 * urand(),
                    .base = 160 + 30 * urand(),
                    .noise = 3 + 6 * urand(),
                    .scale = 0.85 + 0.3 * urand(),
                    .speed = 20 + 80 * urand(),
                    .stop_prob = 0.01};
        double u = 2.6 + 0.8 * urand();
        tune(&o, u, 8 + 30 * urand(), 600 + 1800 * urand());
        Iv iv[16];
        unsigned n = label(c, 40, u, t & 1, iv);
        char got[4];
        unsigned groups;
        BpResult last;
        unsigned k = drive(iv, n, 40 + 47 * u + 60, &o, got, 4, &last, &groups);
        ++trials;
        if (k == 1 && got[0] == c)
            ++ok;
        for (unsigned j = 0; j < k; ++j)
            if (got[j] != c)
                ++mis;
    }
    printf("harsh: %u/%u decoded, %u misreads\n", ok, trials, mis);
    CHECK(mis == 0,
          "harsh conditions (low contrast, high noise, speed/scale extremes): no misreads");
}

/* ---------- D: things that are not barcodes ---------- */
static void test_negatives(void) {
    Optics o = {.sigma = 1.8, .H = 40, .base = 175, .noise = 3, .scale = 1, .speed = 45};
    tune(&o, 3, 35, 1800);
    char got[4];
    unsigned groups, k;
    BpResult last;
    k = drive(NULL, 0, 1000, &o, got, 4, &last, &groups);
    CHECK(k == 0 && groups == 0, "1 m of white: nothing");
    Iv j[1] = {{100, 120}};
    k = drive(j, 1, 300, &o, got, 4, &last, &groups);
    CHECK(k == 0 && groups == 0, "20 mm junction stripe: not a barcode, not reported");
    Iv two[2] = {{100, 120}, {300, 320}};
    k = drive(two, 2, 500, &o, got, 4, &last, &groups);
    CHECK(k == 0, "two junction stripes: nothing");
    /* truncated label: only the first 10 bars (*X) */
    Iv iv[16];
    unsigned n = label('Z', 40, 3, false, iv);
    k = drive(iv, 10, 260, &o, got, 4, &last, &groups);
    CHECK(k == 0 && last.reason == BP_R_FEW, "truncated *Z: rejected as too few bars");
    /* missing one narrow bar in the middle character */
    n = label('A', 40, 3, false, iv);
    for (unsigned i = 7; i + 1 < n; ++i)
        iv[i] = iv[i + 1];
    k = drive(iv, n - 1, 260, &o, got, 4, &last, &groups);
    CHECK(k == 0, "one bar missing: rejected, not guessed");
    /* an extra narrow bar inside a wide space */
    n = label('F', 40, 3, false, iv);
    unsigned m = n;
    double gap_a = 0, gap_b = 0;
    for (unsigned i = 0; i + 1 < n; ++i)
        if (iv[i + 1].a - iv[i].b > 8) {
            gap_a = iv[i].b;
            gap_b = iv[i + 1].a;
            break;
        }
    iv[m++] = (Iv){0.5 * (gap_a + gap_b) - 1.5, 0.5 * (gap_a + gap_b) + 1.5};
    for (unsigned i = m - 1; i > 0 && iv[i].a < iv[i - 1].a; --i) {
        Iv t = iv[i];
        iv[i] = iv[i - 1];
        iv[i - 1] = t;
    }
    k = drive(iv, m, 260, &o, got, 4, &last, &groups);
    CHECK(k == 0, "extra bar in a wide space: rejected");
    /* random 15-bar noise patterns never produce a frame unless they happen to be valid */
    unsigned false_frames = 0;
    for (unsigned t = 0; t < 300; ++t) {
        Iv r[15];
        double x = 40;
        for (unsigned i = 0; i < 15; ++i) {
            double w = urand() < 0.4 ? 9 : 3;
            r[i] = (Iv){x, x + w};
            x += w + (urand() < 0.3 ? 9 : 3);
        }
        k = drive(r, 15, x + 60, &o, got, 4, &last, &groups);
        false_frames += k;
    }
    printf(
        "random bar patterns: %u frames accepted out of 300 (only exact *X* patterns can pass)\n",
        false_frames);
    CHECK(false_frames <= 3, "random bar patterns rarely form a valid *X* frame");
    /* overflow */
    Iv many[30];
    for (unsigned i = 0; i < 30; ++i)
        many[i] = (Iv){40 + 6.0 * i, 43 + 6.0 * i};
    BarcodePeak p;
    bp_reset(&p);
    bool ovf = false;
    unsigned dec = 0;
    for (double x = 0; x < 300; x += 0.1) {
        BpResult r;
        double a = adc_at(many, 30, x, &o) + o.noise * nrand();
        if (bp_sample(&p, (uint16_t)(a + 0.5), x, &r)) {
            ovf |= r.overflow;
            dec += r.decoded;
        }
    }
    CHECK(ovf && dec == 0, "30 bars: overflow flagged, no decode");
}

/* ---------- E: sensor/odometry quirks ---------- */
static void test_quirks(void) {
    BarcodePeak p;
    bp_reset(&p);
    BpResult r;
    bool any = false;
    for (unsigned k = 0; k < 5000; ++k)
        any |=
            bp_sample(&p, (uint16_t)(175 + (k % 7)), 10.0 - 0.001 * k, &r); /* odometry backwards */
    CHECK(!any && p.last_mm == 10.0, "path never steps backwards; stationary noise makes no bars");
    bp_reset(&p);
    any = false;
    for (unsigned k = 0; k < 20000; ++k)
        any |= bp_sample(&p, (uint16_t)(k & 1 ? 220 : 175), 50.0, &r); /* stopped on noisy edge */
    CHECK(!any, "stopped car: nothing reported (no travel)");
}

/* ---------- G: latency and a junction close behind the label ---------- */
static void test_latency(void) {
    Optics o = {.sigma = 1.8, .H = 40, .base = 175, .noise = 2.5, .scale = 1, .speed = 45};
    tune(&o, 3, 35, 1800);
    unsigned worst_ok = 1;
    double worst = 0;
    for (unsigned t = 0; t < 40; ++t) {
        char c = "AFZY"[t % 4];
        Iv iv[17];
        unsigned n = label(c, 40, 3, t & 1, iv);
        double last_edge = iv[n - 1].b;
        iv[n] = (Iv){last_edge + 30, last_edge + 48}; /* 18 mm junction 30 mm after the label */
        BarcodePeak p;
        bp_reset(&p);
        unsigned dec = 0;
        char got = 0;
        double at = 0;
        for (double x = 0; x < last_edge + 120; x += 0.1) {
            BpResult r;
            double a = adc_at(iv, n + 1, x, &o) + o.noise * nrand();
            if (bp_sample(&p, (uint16_t)(a + 0.5), x, &r) && r.decoded) {
                ++dec;
                got = r.code;
                at = x;
            }
        }
        if (dec != 1 || got != c)
            worst_ok = 0;
        if (at - last_edge > worst)
            worst = at - last_edge;
    }
    printf(
        "latency: decoded at most %.1f mm after the label's last edge (junction stripe 30 mm behind)\n",
        worst);
    CHECK(worst_ok, "label then junction 30 mm later: one correct decode, junction not merged");
    CHECK(worst < 20.0, "decode within 20 mm of the label's last edge (early path)");
}

/* ---------- F: telemetry ---------- */
static void test_tlm(void) {
    BpResult r = {.decoded = true,
                  .reverse = true,
                  .code = 'Z',
                  .reason = BP_OK,
                  .nbars = 15,
                  .offset = 0,
                  .start_mm = 100,
                  .end_mm = 241.5,
                  .unit_mm = 3.0,
                  .base = 175,
                  .max_narrow_space = 1.25,
                  .min_wide_space = 3.05,
                  .min_narrow_rise = 31,
                  .min_narrow_peak = 206,
                  .max_wide_peak = 2287};
    uint8_t f[BP_TLM_FRAME];
    bp_tlm_pack(&r, 0x01020304u, 0x13, 7, 0x5a, f);
    CHECK(f[0] == 0xF2 && f[1] == 24 && f[2] == 0x5a && sizeof f == 27,
          "source 15 header, 24-byte payload");
    CHECK(f[3] == 4 && f[6] == 1 && f[7] == 0xB1 && f[8] == 0x13 && f[9] == 'Z' && f[11] == 15,
          "fields");
    CHECK(f[13] == 175 && f[15] == 206 && (f[17] | f[18] << 8) == 2287 && f[19] == 31 &&
              f[20] == 25 && f[21] == 61 && f[22] == 150,
          "stats");
    CHECK((f[23] | f[24] << 8) == 1415 && (f[25] | f[26] << 8) == 7, "span and group count");
    for (unsigned i = 0; i < sizeof f; ++i)
        printf("%02x", f[i]);
    printf("\n");
}

int main(void) {
    test_captures();
    test_synthetic();
    test_harsh();
    test_negatives();
    test_quirks();
    test_latency();
    test_tlm();
    if (fails) {
        printf("FAILED %d of %d checks\n", fails, checks);
        return 1;
    }
    printf(
        "PASS barcode peak decoder v0.1 host tests: %d checks (4 captured car failures, synthetic A-Z/0-9 both directions, harsh no-misread, negatives, telemetry)\n",
        checks);
    return 0;
}
