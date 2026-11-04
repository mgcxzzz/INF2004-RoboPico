/* Compile the real formatter and ring append policy; no RTOS/hardware I/O.
 * No formatter lock stubs are provided: a return to whole-format locking
 * fails compilation/linking. Only the complete-record sink masks here. */
#define APP_RTOS_IMU 1
#include <tk/tkernel.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "lib/libtm/sysdepend/pico_rp2040/usb/usb_record.h"
#include "lib/libtm/libtm_printf.c"

static UB ring[1024];
static volatile UW head;
static UW tail, calls, drops, oversized;
static unsigned masked, inject;
void tm_usb_console_record(const UB *data, INT size) {
    assert(!masked); /* reached with caller interrupts still enabled */
    if (inject) {
        inject = 0;
        tm_printf((const UB *)"nested=%u\n", 7U);
    }
    masked = 1;
    ++calls;
    if (!data || !tm_usb_record_append(ring, 1023, &head, tail, data, size)) {
        ++drops;
        oversized += !data;
    }
    masked = 0;
}
static void expect(const char *text) {
    for (unsigned i = 0; text[i]; ++i) {
        assert(tail != head && ring[tail] == (UB)text[i]);
        tail = (tail + 1U) & 1023;
    }
    assert(tail == head && !masked);
}
int main(void) {
    int n = tm_printf((const UB *)"value=%d unsigned=%u hex=%08x str=%s c=%c %%\n", -12, 42U,
                      0x12abU, (UB *)"hello", 'Q');
    const char *text = "value=-12 unsigned=42 hex=000012ab str=hello c=Q %\r\n";
    assert(n == (int)strlen(text) - 1 && calls == 1 && !drops);
    expect(text);
    inject = 1;
    tm_printf((const UB *)"outer=%u\n", 123U);
    expect("nested=7\r\nouter=123\r\n");

    /* Wrap, exact768-byte boundary and oversize drop without partial text. */
    head = tail = 900;
    UB s[770];
    memset(s, 'x', 768);
    s[768] = 0;
    assert(tm_printf((const UB *)"%s", s) == 768);
    expect((const char *)s);
    s[768] = 'x';
    s[769] = 0;
    tm_printf((const UB *)"%s", s);
    assert(drops == 1 && oversized == 1 && head == tail);

    /* CRLF expansion also respects cap and the printf return counts input. */
    memset(s, '\n', 384);
    s[384] = 0;
    assert(tm_printf((const UB *)"%s", s) == 384);
    for (unsigned i = 0; i < 384; ++i) {
        assert(ring[tail] == '\r');
        tail = (tail + 1U) & 1023;
        assert(ring[tail] == '\n');
        tail = (tail + 1U) & 1023;
    }
    assert(head == tail);
    s[384] = '\n';
    s[385] = 0;
    tm_printf((const UB *)"%s", s);
    assert(drops == 2 && oversized == 2 && head == tail);

    /* A full ring discards the complete message and changes no prefix. */
    head = 1020;
    tail = 0;
    UB before[sizeof ring];
    memcpy(before, ring, sizeof ring);
    tm_printf((const UB *)"NO PARTIAL LINE\n");
    assert(head == 1020 && !memcmp(before, ring, sizeof ring));
    assert(drops == 3 && oversized == 2);
    puts(
        "PASS actual USB printf: no formatting lock, nested producer integrity, CRLF, wrap,768-byte limit and whole-record full/oversize drops.");
}
