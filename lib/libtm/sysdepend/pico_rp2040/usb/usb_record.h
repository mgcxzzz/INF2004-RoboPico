#ifndef TM_USB_RECORD_H
#define TM_USB_RECORD_H
/* Caller owns the USB ring lock. Preflight before copying so a full ring
 * cannot leave a partial printf line. Head is published only after copy. */
static inline INT tm_usb_record_append(UB *ring, UW mask, volatile UW *head, UW tail,
                                       const UB *data, INT size)
{
    UW next = *head;
    if(size < 0 || data == 0 || (UW)size > ((tail - next - 1U) & mask)) return 0;
    while(size-- > 0) {
        ring[next] = *data++;
        next = (next + 1U) & mask;
    }
    *head = next;
    return 1;
}
#endif
