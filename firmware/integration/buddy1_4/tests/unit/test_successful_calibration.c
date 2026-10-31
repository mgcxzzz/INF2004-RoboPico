#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "config/calibration_presets.h"
#include "src/barcode/barcode_record.h"
static void seal(BarcodeSaved *saved) {
    saved->crc = log_crc((const uint8_t *)saved + offsetof(BarcodeSaved, count),
                         barcode_record_crc_size(saved->version));
}
int main(void) {
    LineCalibration old = {.white = {99, 100}, .black = {800, 900}, .mask = 7};
    LineCalibration good = successful_calibration(), chosen = old;
    assert(calibration_valid(&good));
    assert(good.white[0] == 163 && good.white[1] == 160 && good.black[0] == 3457 &&
           good.black[1] == 3700);
    assert(successful_calibration_boot(&chosen, 26, true));
    assert(memcmp(&chosen, &good, sizeof(good)) == 0);
    chosen = old;
    assert(successful_calibration_boot(&chosen, 27, true));
    assert(memcmp(&chosen, &good, sizeof(good)) == 0);
    chosen = old;
    assert(!successful_calibration_boot(&chosen, 28, true));
    assert(memcmp(&chosen, &old, sizeof(old)) == 0);
    assert(successful_calibration_boot(&chosen, 28, false));
    assert(memcmp(&chosen, &good, sizeof(good)) == 0);
    BarcodeSaved saved = {.magic = B_MAGIC,
                          .version = 28,
                          .profile = 1,
                          .recorded_profile = 1,
                          .calibration = old,
                          .side = {0, 1, 3}};
    seal(&saved);
    assert(barcode_record_valid(&saved));
    saved.version = 26;
    seal(&saved);
    assert(barcode_record_valid(&saved));
    saved.version = 27;
    seal(&saved);
    assert(barcode_record_valid(&saved));
    saved.version = 28;
    seal(&saved);
    saved.crc ^= 1;
    assert(!barcode_record_valid(&saved));
    chosen = successful_calibration(); /* c restores without modifying the preset */
    chosen.black[0] = 2200;
    assert(successful_calibration().black[0] == 3457);
    puts(
        "PASS calibration preset, legacy override, saved-custom restore, CRC rejection and immutable restore source.");
}
