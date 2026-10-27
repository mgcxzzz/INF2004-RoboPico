/* Included in the standalone hardware main. z is independent of p/j. */
#include "nav_log_storage.h"
#define INTERFACE_LOG_OFFSET (RAW_OFFSET-NAV_LOG_FLASH_BYTES)
static bool interface_log_failed;
static void interface_save_nav_log(void) {
    if(!runs)return; /* k after a reboot must not erase the previous API run. */
    if((uintptr_t)&__flash_binary_end>XIP_BASE+INTERFACE_LOG_OFFSET) {interface_log_failed=true;return;}
    unsigned count=motion_log_count(),overwritten=motion_log_overwritten();
    uint8_t header[NAV_LOG_HEADER],packet[NAV_EVENT_BYTES],page[256];MotionEvent e;
    uint32_t crc=UINT32_MAX;
    for(unsigned n=0;n<count;++n) {if(!motion_log_read(n,&e)) {interface_log_failed=true;return;}
        nav_event_encode(packet,&e);crc=nav_crc_update(crc,packet,sizeof(packet));}
    nav_log_header(header,count,overwritten,guard.start_us,guard.duration_us,guard.reason,~crc);
    uint32_t irq=save_and_disable_interrupts();
    flash_range_erase(INTERFACE_LOG_OFFSET,NAV_LOG_FLASH_BYTES);
    unsigned last_row=UINT32_MAX;
    for(unsigned offset=0;offset<count*NAV_EVENT_BYTES;offset+=256) {
        memset(page,0xff,sizeof(page));
        for(unsigned k=0;k<256 && offset+k<count*NAV_EVENT_BYTES;++k) {
            unsigned row=(offset+k)/NAV_EVENT_BYTES;
            if(row!=last_row) {motion_log_read(row,&e);nav_event_encode(packet,&e);last_row=row;}
            page[k]=packet[(offset+k)%NAV_EVENT_BYTES];
        }
        flash_range_program(INTERFACE_LOG_OFFSET+NAV_LOG_HEADER+offset,page,sizeof(page));
    }
    /* Header last: interrupted writes fail header/data CRC rather than look valid. */
    flash_range_program(INTERFACE_LOG_OFFSET,header,sizeof(header));restore_interrupts(irq);
    interface_log_failed=!nav_log_valid((const uint8_t *)(XIP_BASE+INTERFACE_LOG_OFFSET));
}
static const char *interface_kind(unsigned k) {
    static const char *const names[]={"TAKEOVER","STOP","DRIVE_MM","TURN_DEG","RESUME_LINE","BARCODE","TERRAIN","LINE_TARGETS","SPEED_ASSIST","FAULT_REPORT"};
    return k<10?names[k]:"UNKNOWN";
}
static const char *interface_event(unsigned k) {
    static const char *const names[]={"NONE","PAUSE_REQUEST","STOPPED","START","DONE","CANCELLED","FAILED","RESUMED","ROUTE_ACCEPTED","TERRAIN_REPORTED","REJECTED","FAULT_REPORTED","TARGETS_ACCEPTED","NAV_STATE","SESSION_END"};
    return k<15?names[k]:"UNKNOWN";
}
static void interface_dump_nav_log(void) {
    if(guard.active || trace_settling) {puts("NAV_LOG: stop and wait5s before z; printing while moving would disrupt control.");return;}
    const uint8_t *bank=(const uint8_t *)(XIP_BASE+INTERFACE_LOG_OFFSET);bool restored=!runs;
    if(restored && !nav_log_valid(bank)) {puts("NAV_LOG: no CRC-verified coordinator log saved yet. Normal p/j logs are separate.");return;}
    unsigned count=restored?bank[5]:motion_log_count(),overwritten=restored?nav_rd32(bank+8):motion_log_overwritten();
    uint64_t start=restored?nav_rd64(bank+24):guard.start_us,duration=restored?nav_rd64(bank+32):guard.duration_us;
    printf("NAV_LOG_BEGIN: rows=%u overwritten=%u restored=%u duration_ms=%" PRIu64 " stop_reason=%u matches_motion=%u stride=%u saved=%u\n",
        count,overwritten,restored,duration/1000,restored?bank[6]:guard.reason,duration==guard.duration_us,NAV_EVENT_BYTES,!interface_log_failed);
    puts("seq,ms,requester_buddy,controller_owner,command_id,event,request,goal,speed_mm_s,style,result,state,fault_bitmap,progress_mm,left_rotation_deg,targetL_mm_s,targetR_mm_s,overflow");
    for(unsigned n=0;n<count;++n) {
        MotionEvent e;bool overflow=false;
        if(restored) {if(!nav_event_decode(&e,bank+NAV_LOG_HEADER+n*NAV_EVENT_BYTES,&overflow))continue;}
        else if(!motion_log_read(n,&e))continue;
        printf("%lu,%" PRIu64 ",%u,%u,%lu,%s,%s,%.1f,%u,%s,%s,%u,0x%03x,%.1f,%.1f,%d,%d,%u\n",
            (unsigned long)e.sequence,(e.now_us>=start?e.now_us-start:0)/1000,e.requester,e.owner,(unsigned long)e.id,
            interface_event(e.type),interface_kind(e.kind),(double)e.goal,e.speed,e.style?"IN_PLACE":"ROLLING",
            motion_result_name((MotionResult)e.result),e.state,e.fault,(double)e.progress_mm,(double)e.rotation_deg,e.target_mm_s[0],e.target_mm_s[1],overflow);
        stdio_flush();sleep_ms(5);
    }
    puts("NAV_LOG_END. Request accepted is not movement completed; wait for DONE or query command_id.");
}
