#include <assert.h>
#include <stdio.h>
#include "src/telemetry/drive_diag_storage.h"
static DriveDiag capture, restored;
static uint8_t bank[DIAG_BANK];
static void snapshot(void) {
    memset(bank, 255, sizeof(bank));
    diag_header(bank, &capture);
    memcpy(bank + DIAG_HEADER, capture.rows, sizeof(capture.rows));
}
static void codec(void) {
    assert(diag_bits() <= DIAG_ROW_BYTES * 8 && diag_bits() > (DIAG_ROW_BYTES - 1) * 8);
    for (unsigned field = 0; field < D_FIELDS; ++field) {
        int64_t lo = diag_signed[field] ? -(INT64_C(1) << (diag_width[field] - 1)) : 0;
        int64_t hi = (INT64_C(1) << (diag_width[field] - diag_signed[field])) - 1;
        int64_t checks[] = {lo, lo + 1, 0, hi - 1, hi};
        for (unsigned k = 0; k < 5; ++k) {
            DiagValues v = {0}, out = {0};
            DiagRow r;
            v.v[field] = checks[k];
            diag_pack(&r, &v);
            diag_unpack(&r, &out);
            for (unsigned j = 0; j < D_FIELDS; ++j)
                assert(v.v[j] == out.v[j]);
        }
        DiagValues v = {0}, out;
        DiagRow r;
        v.v[field] = hi + 1;
        diag_pack(&r, &v);
        diag_unpack(&r, &out);
        assert(out.v[D_FLAGS] & DF_CLIPPED);
        assert(out.v[field] == hi || field == D_FLAGS);
    }
    puts(
        "PASS minimum-width diagnostic codec: all field extrema, sign extension, quantisation and clipping flag.");
}
static void capture_and_flash(void) {
    const uint16_t white[] = {163, 160}, black[] = {3457, 3700};
    diag_begin(&capture, 1234567, 1, white, black);
    DiagValues v = {0};
    v.v[D_DT] = 10000;
    v.v[D_PROFILE] = 1;
    v.v[D_PHASE] = 10;
    v.v[D_TARGET_L] = v.v[D_TARGET_R] = 45;
    v.v[D_PWM_L] = v.v[D_PWM_R] = 150;
    for (unsigned k = 0; k < 600; ++k) {
        v.v[D_US] = (k + 1) * 10000;
        v.v[D_PWM_R] = k >= 299 ? 300 : 150;
        int slot = diag_push(&capture, &v);
        diag_cost(&capture, slot, k == 550 ? 1234 : 450);
    }
    assert(capture.triggered && capture.frozen && capture.trigger == 1 &&
           capture.trigger_us == 3000000);
    assert(capture.count == 256 && capture.total == 600 && capture.max_cost == 1234 &&
           capture.max_gap == 10000 && !capture.late20);
    DiagValues first, last;
    diag_unpack(&capture.rows[(capture.head + DIAG_ROWS - capture.count) % DIAG_ROWS], &first);
    diag_unpack(&capture.rows[(capture.head + DIAG_ROWS - 1) % DIAG_ROWS], &last);
    assert(first.v[D_US] == 1730000 && last.v[D_US] == 4280000);
    capture.duration_us = 6000000;
    capture.ticks[0] = -456;
    capture.ticks[1] = 567;
    capture.reason = 11;
    snapshot();
    assert(diag_bank_valid(bank));
    diag_restore(&restored, bank);
    assert(restored.count == capture.count && restored.start_us == capture.start_us &&
           restored.ticks[0] == -456);
    assert(restored.white[0] == 163 && restored.black[1] == 3700 &&
           !memcmp(restored.rows, capture.rows, sizeof(capture.rows)));
    for (unsigned n = 0; n < DIAG_HEADER + sizeof(capture.rows); n += 43) {
        bank[n] ^= 1;
        assert(!diag_bank_valid(bank));
        bank[n] ^= 1;
    }
    memset(bank, 255, DIAG_HEADER);
    assert(!diag_bank_valid(bank));
    snapshot();
    bank[DIAG_HEADER + 89] ^= 8;
    assert(!diag_bank_valid(bank));
    /* A short manually stopped capture before a trigger is still useful. */
    diag_begin(&capture, 7, 3, white, black);
    v.v[D_PROFILE] = 3;
    v.v[D_US] = 10000;
    diag_cost(&capture, diag_push(&capture, &v), 300);
    snapshot();
    assert(diag_bank_valid(bank));
    /* If no trigger occurs, retain the latest2.56s; report clipped timing. */
    diag_begin(&capture, 8, 1, white, black);
    v.v[D_PROFILE] = 1;
    for (unsigned k = 0; k < 700; ++k) {
        v.v[D_US] = (k + 1) * 10000;
        diag_push(&capture, &v);
    }
    assert(!capture.triggered && !capture.frozen && capture.count == 256);
    diag_cost(&capture, (capture.head + 255) % 256, 200000);
    snapshot();
    assert(diag_bank_valid(bank));
    diag_unpack(&capture.rows[(capture.head + 255) % 256], &last);
    assert(last.v[D_COST] == 131071 && (last.v[D_FLAGS] & DF_CLIPPED));
    /* A preserved-cruise event is captured even when its PWM is now smooth. */
    diag_begin(&capture, 9, 1, white, black);
    v.v[D_US] = 2090000;
    v.v[D_FLAGS] = 0;
    diag_push(&capture, &v);
    v.v[D_US] = 2100000;
    v.v[D_FLAGS] = DF_HANDOVER_KEEP;
    diag_push(&capture, &v);
    assert(capture.triggered && capture.trigger == 4);
    snapshot();
    assert(diag_bank_valid(bank));
    diag_restore(&restored, bank);
    assert(restored.trigger == 4 && restored.triggered);
    bank[4] = 3;
    memset(bank + 16, 0, 4);
    nav_le32(bank + 16, nav_crc(bank, DIAG_HEADER));
    assert(!diag_bank_valid(bank));
    /* New controller has intentionally small per-tick changes; still capture it. */
    diag_begin(&capture, 10, 1, white, black);
    v.v[D_FLAGS] = 0;
    v.v[D_DRIVER] = 3;
    v.v[D_PWM_L] = v.v[D_PWM_R] = 150;
    for (unsigned k = 0; k < 450; ++k) {
        v.v[D_US] = (k + 1) * 10000;
        diag_push(&capture, &v);
    }
    assert(capture.triggered && capture.trigger == 4 && capture.trigger_us == 3000000 &&
           capture.frozen);
    snapshot();
    assert(diag_bank_valid(bank));
    puts(
        "PASS triggered pre/post ring, latest-window fallback, whole-run timing after freeze, interrupted/corrupt flash rejection, reboot restore and short-run storage.");
}
int main(void) {
    codec();
    capture_and_flash();
    printf("DIAG row=%zu bits=%u RAM=%zu flash=%u\n", sizeof(DiagRow), diag_bits(),
           sizeof(DriveDiag), DIAG_BANK);
}
