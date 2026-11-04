#ifndef RTOS_LINE_MODEL_H
#define RTOS_LINE_MODEL_H

/* Hardware-free Step 2 adapter. The reference algorithms remain in main's
 * feature folders; only the RTOS acquisition/output boundary lives here. */
#ifdef APP_RTOS_BARCODE
/* Horizontal GP27 sensing point -> wheel axle, measured on the current car.
 * Other migration steps and the SDK reference retain their 92mm geometry. */
#define BARCODE_SENSOR_AHEAD_MM 110
#ifndef BARCODE_CONFIRM_GATE
#define BARCODE_CONFIRM_GATE 1
#endif
#endif
#include "src/line_following/continuous_follow.h"
#include "src/line_following/circuit_capture.h"
#include "src/line_following/circuit_load.h"
#include "src/motion/maneuver_power.h"
#include "src/motion/ramp_profile.h"
#include "src/barcode/barcode_record.h"
#include "config/calibration_presets.h"

enum {
    RL_PROFILE = 1,
    RL_CAL_OFFSET = 0x1a9000,
    RL_MAIN_OFFSET = 0x1d0000,
    RL_CAL_MAGIC = 0x31434c52,
    RL_LOG_ROWS =
#ifdef APP_RTOS_BARCODE
        640 /* RAM is shared with the original63 banks and2ms raw capture. */
#else
        1536
#endif
};

enum {
    RL_J_WHITE_NOW = 1,
    RL_J_WHITE_QUALIFIED = 2,
    RL_J_DEPARTURE_READY = 4,
    RL_J_BRANCH_ACCEPTED = 8,
    RL_J_ACCEPTED_WITHOUT_WHITE = 16,
    RL_J_ACCEPT_EVENT = 32,
    RL_J_WHITE_TIMING = 64
};
typedef struct {
    uint64_t white_us;
    uint8_t flags;
} LineJunctionDiagnostic;

typedef struct {
    Circuit circuit;
    BarcodeNav nav;
    ContinuousSteering steering;
    ScanSmooth scan;
    AnalogDemand optical;
    uint16_t baseline, side_adc;
    uint64_t candidate_us;
    double candidate_wheel[2];
    bool black, candidate;
    LineJunctionDiagnostic junction;
} LinePlanner;

/* Observation only: never modifies the planner's acceptance or motor targets.
 * White qualification reproduces the former <200, >10deg, 30ms condition.
 * For U-turns, acceptance is stage2 -> stage3, before the second settle. */
static inline void rl_junction_observe(LinePlanner *p, const LineCalibration *cal,
                                       const CircuitInput *i, unsigned before, unsigned stage) {
    LineJunctionDiagnostic *q = &p->junction;
    const Junction *j = &p->circuit.turn;
    if (before == J_CENTRE && p->circuit.phase == J_TURN)
        *q = (LineJunctionDiagnostic){0};
    q->flags &= (uint8_t) ~(RL_J_WHITE_NOW | RL_J_ACCEPT_EVENT);
    bool white = line_level(i->adc[0], cal->white[0], cal->black[0]) < 200 &&
                 line_level(i->adc[1], cal->white[1], cal->black[1]) < 200;
    if (white)
        q->flags |= RL_J_WHITE_NOW;
    if (before == J_TURN && j->turn_degrees > 10 && white) {
        if (!(q->flags & RL_J_WHITE_TIMING)) {
            q->white_us = i->now;
            q->flags |= RL_J_WHITE_TIMING;
        }
        if (i->now - q->white_us >= 30000)
            q->flags |= RL_J_WHITE_QUALIFIED;
    } else
        q->flags &= (uint8_t)~RL_J_WHITE_TIMING;
    if (j->off_line)
        q->flags |= RL_J_DEPARTURE_READY;
    bool uturn = barcode_nav_action(&p->nav, p->nav.action) == 'U';
    bool accepted = before == J_TURN &&
                    (uturn ? stage == 2 && p->nav.uturn.stage == 3 : p->circuit.phase == J_ACQUIRE);
    if (accepted) {
        q->flags |= RL_J_BRANCH_ACCEPTED | RL_J_ACCEPT_EVENT;
        if (!(q->flags & RL_J_WHITE_QUALIFIED))
            q->flags |= RL_J_ACCEPTED_WITHOUT_WHITE;
    }
}

static inline void rl_begin(LinePlanner *p, const CircuitInput *i, uint16_t side_adc) {
    *p = (LinePlanner){.baseline = side_adc,
                       .side_adc = side_adc,
                       .black = side_adc >= 212,
                       .candidate = side_adc >= 212};
    circuit_begin(&p->circuit, i);
    barcode_nav_begin(&p->nav, i);
    p->nav.external_reader = true; /* Code39 is deliberately deferred to Step 4. */
}

/* Junction-only edge qualification. Same 212 ADC preview, 3 ADC hysteresis,
 * 2 ms debounce, encoder width 11..30 mm and 4 mm white confirmation as the
 * reference. No call to either barcode decoder; test with a bare cross. */
static inline void rl_side(LinePlanner *p, uint16_t adc, uint64_t now, const double wheel[2]) {
    p->side_adc = adc;
    bool candidate = p->black ? adc >= 209 : adc >= 215;
    if (candidate != p->candidate) {
        p->candidate = candidate;
        p->candidate_us = now;
        memcpy(p->candidate_wheel, wheel, sizeof(p->candidate_wheel));
    }
    if (candidate == p->black || now - p->candidate_us < AO_DEBOUNCE_US)
        return;
    p->black = candidate;
    BarcodeNav *n = &p->nav;
    if (n->busy)
        return;
    double mm = (p->candidate_wheel[0] + p->candidate_wheel[1]) * .5;
    if (candidate) {
        barcode_cross_confirm(n, p->candidate_us, mm, true);
        n->cross_candidate = false;
        memcpy(n->leading_wheel, p->candidate_wheel, sizeof(n->leading_wheel));
        n->leading_us = p->candidate_us;
        n->leading_gate = n->allow_cross;
    } else if (n->leading_us) {
        double width = mm - (n->leading_wheel[0] + n->leading_wheel[1]) * .5;
        if (width >= B_CROSS_MIN_MM && width <= B_CROSS_MAX_MM) {
            n->cross_width = width;
            n->cross_us = n->leading_us;
            memcpy(n->cross_wheel, n->leading_wheel, sizeof(n->cross_wheel));
            n->cross_gate = n->leading_gate;
            n->cross_candidate = true;
            n->cross_trailing_mm = mm;
        }
        n->leading_us = 0;
    }
}

/* Profile 1 part of continuous_follow_step, without allocating 63 inactive
 * decoder banks. Host tests compare it against the unchanged full wrapper. */
static inline AnalogDemand rl_follow(LinePlanner *p, const LineCalibration *cal,
                                     const CircuitInput *i, float dt) {
    unsigned before = p->circuit.phase, stage = p->nav.uturn.stage;
    AnalogDemand d = barcode_circuit_step(&p->circuit, &p->nav, cal, i, RL_PROFILE);
    rl_junction_observe(p, cal, i, before, stage);
    p->optical = d;
    bool activity = p->nav.pending || p->side_adc >= (unsigned)p->baseline + 16;
    if (activity)
        p->scan.activity_us = i->now;
    bool scanning = !p->circuit.busy && !d.stop &&
                    (activity || (p->scan.scanning && i->now - p->scan.activity_us < 600000));
    p->scan.scanning = scanning;
    return continuous_steering_step(&p->steering, d, &p->circuit, i, scanning, RL_PROFILE, dt);
}

typedef struct {
    ContinuousDrive drive[2];
    SignedWheel wheel[2];
    ManeuverPower maneuver[2];
    CircuitCapture capture;
    CircuitLoad load;
    int32_t output[2];
    unsigned assist_code;
} LinePower;

/* A committed envelope is copied under the motion lock. The 1 ms safety
 * task can expire peaks without reading the controller's mutable PI state. */
typedef struct {
    int32_t output[2];
    uint64_t until[2];
    unsigned continuous : 1, maneuver : 1, reverse : 1;
    unsigned loaded : 2;
} LineEnvelope;

static inline LineEnvelope rl_power(LinePower *s, const LinePlanner *p, const CircuitInput *i,
                                    const AnalogDemand *d, unsigned before, float dt) {
    const Circuit *c = &p->circuit;
    if ((before == J_TURN ||
         (barcode_nav_action(&p->nav, p->nav.action) == 'U' && before == J_TURN_SETTLE)) &&
        c->phase == J_ACQUIRE)
        circuit_capture_begin(&s->capture, i->now, i->path);
    bool turning = c->phase == J_TURN && ((d->target[0] == 0 && d->target[1] > 0) ||
                                          (d->target[1] == 0 && d->target[0] != 0));
    const CircuitTracker *tracker = c->phase == J_ACQUIRE ? &c->joining : &c->follower;
    bool recovering = tracker->core.white;
    int capture = circuit_capture_outer(&s->capture, i, c->phase, d);
    int search = line_search_outer(tracker->search.active && recovering && !d->stop, d->target[0],
                                   d->target[1]);
    bool maneuver = (turning && barcode_nav_action(&p->nav, p->nav.action) == 'U') || search >= 0;
    if (recovering)
        capture = -1;
    int load = circuit_load_outer(&s->load, i, c->phase, d, capture >= 0 || recovering);
    bool continuous = continuous_drive_selected(RL_PROFILE, c->phase, c->busy, false, false,
                                                maneuver, recovering, d);
    if (continuous) {
        capture = load = -1;
        s->load = (CircuitLoad){0};
    }
    int effort = search >= 0 ? search : capture >= 0 ? capture : load;
    unsigned assist_code = search >= 0    ? (unsigned)search + 5
                           : capture >= 0 ? (unsigned)capture + 1
                           : load >= 0    ? (unsigned)load + 3
                                          : 0;
    LineEnvelope e = {.continuous = continuous,
                      .maneuver = maneuver,
                      .reverse =
                          barcode_nav_action(&p->nav, p->nav.action) == 'U' && c->phase == J_TURN};
    for (unsigned w = 0; w < 2; ++w) {
        bool loaded = turning || effort == (int)w;
        if (loaded)
            e.loaded |= 1U << w;
        if (!continuous)
            continuous_leave(&s->drive[w], &s->wheel[w], w, d->target[w], i->speed[w],
                             s->output[w]);
        if (!maneuver)
            maneuver_leave(&s->maneuver[w], &s->wheel[w], d->target[w]);
    }
    if (!maneuver && effort >= 0 && assist_code != s->assist_code)
        circuit_capture_prepare(&s->wheel[effort]);
    s->assist_code = assist_code;
    for (unsigned w = 0; w < 2; ++w) {
        e.output[w] =
            continuous ? continuous_drive_step(&s->drive[w], &s->wheel[w], w, d->target[w],
                                               i->speed[w], dt, i->now, s->output[w])
            : maneuver ? maneuver_wheel_step(&s->maneuver[w], &s->wheel[w], w, d->target[w],
                                             i->speed[w], dt, i->now)
                       : circuit_wheel_step(&s->wheel[w], w, d->target[w], i->speed[w], dt, i->now,
                                            (e.loaded & (1U << w)) != 0, RL_PROFILE);
        e.until[w] = (maneuver ? s->wheel[w].peak_active : turning) ? s->wheel[w].peak_end_us : 0;
    }
    return e;
}

static inline int32_t rl_cap(const LineEnvelope *e, unsigned w, uint64_t now) {
    return e->continuous ? continuous_output_cap(e->output[w])
           : e->maneuver ? maneuver_output_cap(e->output[w], e->until[w], now)
                         : barcode_output_cap(e->output[w], false, e->until[w], now);
}

static inline DualLevels rl_levels(const LineEnvelope *e, unsigned w, uint64_t now) {
    int32_t output = rl_cap(e, w, now);
    return e->continuous ? continuous_output_levels(w, output)
           : e->maneuver ? maneuver_levels(w, output, e->reverse)
           : e->reverse
               ? boosted_junction_levels(w, output, true)
               : barcode_output_levels(w, output, false, (e->loaded & (1U << w)) != 0, RL_PROFILE);
}

/* A supervisor may only lower live channel levels, never re-enable an API
 * stop or change direction using an older committed controller envelope. */
static inline DualLevels rl_reduce(DualLevels actual, DualLevels cap) {
    return (DualLevels){.a = actual.a < cap.a ? actual.a : cap.a,
                        .b = actual.b < cap.b ? actual.b : cap.b};
}

/* Own sector; never erase the SDK motion log, its calibration or Wi-Fi settings.
 * Fixed-endian page, CRC over all bytes except the first magic+CRC words. */
static inline bool rl_cal_decode(const uint8_t page[256], LineCalibration *cal) {
    uint32_t magic = 0, crc = 0;
    for (unsigned j = 0; j < 4; ++j) {
        magic |= (uint32_t)page[j] << (8 * j);
        crc |= (uint32_t)page[4 + j] << (8 * j);
    }
    if (magic != RL_CAL_MAGIC || page[8] != 1 || crc != log_crc(page + 8, 248))
        return false;
    *cal = (LineCalibration){.mask = page[9] & 7U};
    uint64_t packed = 0;
    for (unsigned n = 0; n < 6; ++n)
        packed |= (uint64_t)page[10 + n] << (8 * n);
    cal->white[0] = (uint16_t)(packed & 4095U);
    cal->white[1] = (uint16_t)((packed >> 12) & 4095U);
    cal->black[0] = (uint16_t)((packed >> 24) & 4095U);
    cal->black[1] = (uint16_t)((packed >> 36) & 4095U);
    return calibration_valid(cal);
}

static inline void rl_cal_encode(uint8_t page[256], const LineCalibration *cal) {
    memset(page, 0xff, 256);
    page[8] = 1;
    page[9] = cal->mask & 7U;
    uint64_t packed = (cal->white[0] & 4095U) | ((uint64_t)(cal->white[1] & 4095U) << 12) |
                      ((uint64_t)(cal->black[0] & 4095U) << 24) |
                      ((uint64_t)(cal->black[1] & 4095U) << 36);
    for (unsigned n = 0; n < 6; ++n)
        page[10 + n] = (uint8_t)(packed >> (8 * n));
    uint32_t crc = log_crc(page + 8, 248);
    for (unsigned j = 0; j < 4; ++j) {
        page[j] = (uint8_t)(RL_CAL_MAGIC >> (8 * j));
        page[4 + j] = (uint8_t)(crc >> (8 * j));
    }
}

static inline LineCalibration rl_boot_cal(const uint8_t page[256], const BarcodeSaved *saved,
                                          uint32_t *origin) {
    LineCalibration cal = successful_calibration();
    *origin = 0;
    if (saved->version == CALIBRATION_RECORD_VERSION && barcode_record_valid(saved)) {
        cal = saved->calibration;
        *origin = 1;
    }
    LineCalibration local;
    if (rl_cal_decode(page, &local)) {
        cal = local;
        *origin = 2;
    }
    return cal;
}

/* 56 bytes/row. ADCs use 12 bits each, calibrated levels 10 each, phase 4;
 * all packed words have one writer and copied snapshots under a spinlock. */
typedef struct {
    uint32_t ms;
    int32_t ticks[2];
    uint32_t optics[2];
    int16_t speed[2], target[2], pwm[2];
    uint16_t dt_us, cost_us;
    uint32_t meta, sequence;
    uint32_t aux[2];
    int16_t optical_target[2];
} LineLogRow;
_Static_assert(sizeof(LineLogRow) == 56, "line log layout");
static inline void rl_pack_optics(LineLogRow *r, const uint16_t adc[3], const AnalogDemand *d,
                                  unsigned phase) {
    r->optics[0] =
        (adc[0] & 4095U) | ((uint32_t)(adc[1] & 4095U) << 12) | ((uint32_t)(adc[2] & 255U) << 24);
    r->optics[1] = ((uint32_t)adc[2] >> 8) | ((uint32_t)(d->level[0] & 1023U) << 4) |
                   ((uint32_t)(d->level[1] & 1023U) << 14) | ((phase & 15U) << 24);
}

static inline void rl_pack_steering(LineLogRow *r, const uint16_t filtered[2], unsigned gain_milli,
                                    unsigned damping_milli) {
    r->aux[0] = (filtered[0] & 4095U) | ((uint32_t)(filtered[1] & 4095U) << 12) |
                ((gain_milli & 255U) << 24);
    r->aux[1] = ((gain_milli >> 8) & 3U) | ((damping_milli & 1023U) << 2);
}

/* Use spare bits: six one-bit observations and signed13-bit angle x10.
 * The 56-byte row and existing steering fields retain their layout. */
static inline void rl_pack_junction(LineLogRow *r, const LinePlanner *p) {
    float degrees = p->circuit.turn.turn_degrees;
    int angle = degrees <= -409.6f  ? -4096
                : degrees >= 409.5f ? 4095
                                    : (int)(degrees * 10 + (degrees < 0 ? -.5f : .5f));
    r->aux[1] = (r->aux[1] & 4095U) | ((uint32_t)(p->junction.flags & 63U) << 12) |
                ((uint32_t)(angle & 8191) << 18);
}
static inline int rl_log_angle(const LineLogRow *r) {
    unsigned value = (r->aux[1] >> 18) & 8191U;
    return value & 4096U ? (int)value - 8192 : (int)value;
}

static inline unsigned rl_route_bits(char route) {
    return route == 'A' ? 1U : route == 'F' ? 2U : route == 'Z' ? 3U : 0U;
}
static inline unsigned rl_route_ascii(unsigned bits) {
    return bits == 1 ? 'A' : bits == 2 ? 'F' : bits == 3 ? 'Z' : 0;
}

#endif
