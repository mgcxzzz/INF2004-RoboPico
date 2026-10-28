/* Included after the hardware/log state. Only core0 touches motion/nav state.
 * Core1 sees copied packets and validated network configuration only. */
#include "wifi_settings.h"
#include "wifi_mqtt_diag.h"
#define WIFI_SETTINGS_OFFSET (PICO_FLASH_SIZE_BYTES-0x56000u)
_Static_assert(WIFI_SETTINGS_OFFSET+FLASH_SECTOR_SIZE==DIAG_OFFSET,"WiFi settings overlap logs");
static bool start_run(unsigned mode);
static unsigned start_block(unsigned mode);
static bool wifi_forward=true,wifi_was_active;
static uint32_t wifi_lease,wifi_seen_barcode,wifi_seen_runs,wifi_app_drops,wifi_summary_run,wifi_summary_connection;
static uint32_t wifi_line_events,wifi_nav_events,wifi_recoveries;
static uint8_t wifi_seq[16],wifi_last_cmd,wifi_last_result=7,wifi_accepted,wifi_rejected;
static uint64_t wifi_last_motion,wifi_last_hb,wifi_start_at;
static bool wifi_download;static WifiCommand wifi_download_command;
static uint8_t wifi_provision_state;static unsigned wifi_provision_size;
static uint8_t wifi_setup_last_code=WIFI_SETUP_NONE;
static char wifi_provision_line[192];

static uint8_t wifi_state(void) {
    if(wifi_estop)return 5;
    MotionStatus s=nav_get_status();
    if(s.mode==MOTION_FAULT || circuit.fault)return 6;
    if(!guard.active)return wifi_stopped?3:0;
    return s.mode==MOTION_STOPPING?3:s.mode==MOTION_PAUSED?2:1;
}
static bool wifi_send(unsigned topic,const uint8_t *p,size_t n) {
    WifiTransportStatus s=wifi_transport_status();
    if(!s.configured || !s.ready)return false;
    return wifi_transport_send(topic,p,n); /* Transport counts its own queue drops exactly once. */
}
static uint16_t wifi_level(unsigned w) {
    if(!calibration_valid(&calibration))return 65535;
    return guard.active?demand.level[w]:line_level(analog_raw[w],calibration.white[w],calibration.black[w]);
}
static int16_t wifi_speed(float value) {return (int16_t)lroundf(clampf(value*10,-32768,32767));}
static int16_t wifi_heading(double left,double right) {
    return (int16_t)lroundf(remainderf(heading_deg((float)left,(float)right),360.0f)*100);
}
static WifiMotionPacket wifi_snapshot(uint64_t now) {
    int64_t ticks[2];uint32_t irq=save_and_disable_interrupts();
    for(unsigned w=0;w<2;++w)ticks[w]=guard.active?total_ticks[w]+enc[w].position:cut_ticks[w];
    uint64_t duration=guard.active?now-guard.start_us:guard.duration_us;
    restore_interrupts(irq);MotionStatus state=nav_get_status();
    WifiMotionPacket p={.uptime_ms=(uint32_t)(now/1000),.elapsed_ms=(uint32_t)(duration/1000),.run_id=(uint16_t)runs,
        .phase=circuit.phase,.profile=test_mode,.flags=(guard.active?1:0)|(calibration_valid(&calibration)?2:0)|
            (state.mode==MOTION_AUTO?4:0)|(wifi_estop?8:0),
        .recovery_state=current_tracker()->core.white?current_tracker()->search.active?2:1:0,
        .speed_left_0p1mm_s=wifi_speed(velocity[0]),.speed_right_0p1mm_s=wifi_speed(velocity[1]),
        .target_left_mm_s=guard.active?demand.target[0]:0,.target_right_mm_s=guard.active?demand.target[1]:0,
        .pwm_left=guard.active?(int16_t)output[0]:0,.pwm_right=guard.active?(int16_t)output[1]:0,
        .adc_left=analog_raw[0],.adc_right=analog_raw[1],.adc_barcode=side_adc,
        .calibrated_level_left=wifi_level(0),.calibrated_level_right=wifi_level(1),
        .distance_mm=(uint32_t)fmin(path_mm(ticks),UINT32_MAX),
        .heading_cdeg=wifi_heading(wheel_mm(0,ticks[0]),wheel_mm(1,ticks[1]))};
    p.line_levels=(calibration_valid(&calibration)?((p.calibrated_level_left>=400?1:0)|(p.calibrated_level_right>=400?2:0)):0)|(side_analog.black?4:0);
    return p;
}
static void wifi_line_status(uint64_t now,unsigned event) {
    uint8_t packet[WPK_LINE_BYTES];
    WifiLinePacket p={.seq=wifi_seq[WPK_SRC_LINE]++,.uptime_ms=(uint32_t)(now/1000),.event=(uint8_t)event,
        .adc_left=analog_raw[0],.adc_right=analog_raw[1],.adc_barcode=side_adc,
        .calibrated_level_left=wifi_level(0),.calibrated_level_right=wifi_level(1),
        .last_decoded_letter=(uint8_t)((nav.last_code>='A'&&nav.last_code<='Z')?nav.last_code:0),
        .pending_letter=(uint8_t)nav.pending,.pending_action=(uint8_t)nav.pending_action,.pending_revision=nav.pending_revision,
        .executing_action=(uint8_t)(nav.busy?nav.executing_action:0),.executing_revision=nav.busy?nav.executing_revision:0};
    if(wpk_line(&p,packet))(void)wifi_send(WIFI_TOPIC_TLM,packet,sizeof(packet));
}
static void wifi_event_id(uint64_t now,unsigned event,char letter,char action,unsigned revision,unsigned detail,uint32_t command_id) {
    ++wifi_nav_events;MotionStatus state=nav_get_status();uint8_t packet[WPK_NAV_BYTES];
    WifiNavPacket p={.seq=wifi_seq[WPK_SRC_NAV]++,.uptime_ms=(uint32_t)(now/1000),.event=(uint8_t)event,
        .letter=(uint8_t)((letter>='A'&&letter<='Z')?letter:0),.action=(uint8_t)action,.map_revision=(uint8_t)revision,
        .command_id=command_id,.generation=state.generation,.fault_bitmap=(uint16_t)state.fault,
        .owner=(uint8_t)state.owner,.mode=(uint8_t)state.mode,.command_state=(uint8_t)state.command_state,.detail=(uint16_t)detail};
    if(wifi_forward && wpk_nav(&p,packet))(void)wifi_send(WIFI_TOPIC_TLM,packet,sizeof(packet));
}
static void wifi_event(uint64_t now,unsigned event,char letter,char action,unsigned revision,unsigned detail) {
    wifi_event_id(now,event,letter,action,revision,detail,0);
}
static void wifi_nav_event(const MotionEvent *e) {
    unsigned event=e->type==MOTION_EVENT_FAULT?WPK_EVENT_FAULT:
        e->type==MOTION_EVENT_DONE?WPK_EVENT_COMPLETED:e->type==MOTION_EVENT_START?WPK_EVENT_DISPATCH:
        e->type==MOTION_EVENT_SESSION_END?WPK_EVENT_STOP:
        e->type==MOTION_EVENT_TARGETS || e->type==MOTION_EVENT_NAV_STATE || e->type==MOTION_EVENT_STOPPED?
        WPK_EVENT_STATUS:WPK_EVENT_COMMAND_RESULT;
    MotionStatus state=nav_get_status();uint8_t packet[WPK_NAV_BYTES];
    WifiNavPacket p={.seq=wifi_seq[WPK_SRC_NAV]++,.uptime_ms=(uint32_t)(e->now_us/1000),.event=(uint8_t)event,
        .command_id=e->id,.generation=state.generation,.fault_bitmap=(uint16_t)e->fault,
        .owner=(uint8_t)e->owner,.mode=(uint8_t)state.mode,.command_state=(uint8_t)e->state,
        .detail=(uint16_t)((e->type<<8)|e->result)};
    ++wifi_nav_events;if(wifi_forward && wpk_nav(&p,packet))(void)wifi_send(WIFI_TOPIC_TLM,packet,sizeof(packet));
}
static void wifi_barcode_events(uint64_t now) {
    if(wifi_seen_runs!=runs){wifi_seen_runs=runs;wifi_seen_barcode=0;wifi_line_events=wifi_nav_events=wifi_recoveries=0;}
    unsigned available=nav.event_sequence-wifi_seen_barcode;
    if(available>nav.event_count){wifi_app_drops+=available-nav.event_count;available=nav.event_count;}
    for(unsigned k=available;k>0;--k) {
        const BarcodeEvent *e=&nav.events[(nav.event_head+B_EVENTS-k)%B_EVENTS];unsigned event;
        switch(e->type) {
        case B_DECODE:event=WPK_EVENT_DECODED;++wifi_line_events;break;
        case B_CROSS:event=WPK_EVENT_JUNCTION_CONFIRMED;break;
        case B_DISPATCH:event=WPK_EVENT_DISPATCH;break;
        case B_COMPLETE:event=WPK_EVENT_COMPLETED;break;
        case B_RECOVERY_WAIT:event=WPK_EVENT_RECOVERY_WAIT;++wifi_recoveries;break;
        case B_RECOVERY_SWEEP:event=WPK_EVENT_RECOVERY_SWEEP;break;
        case B_RECOVERY_FOUND:event=WPK_EVENT_RECOVERY_FOUND;break;
        case B_BAD_FRAME:case B_RECOVERY_FAIL:event=WPK_EVENT_FAULT;break;
        default:continue;
        }
        wifi_event(now,event,(char)e->code,(char)e->action,e->map_revision,e->type==B_BAD_FRAME?e->flags:0);
        if(e->type==B_DECODE)wifi_event(now,e->action=='-'?WPK_EVENT_UNASSIGNED:WPK_EVENT_QUEUED,
            (char)e->code,(char)e->action,e->map_revision,0);
    }
    wifi_seen_barcode=nav.event_sequence;
}
static bool wifi_summary(uint64_t now) {
    int64_t ticks[2]={cut_ticks[0],cut_ticks[1]};uint8_t packet[WPK_SUMMARY_BYTES];
    WifiSummaryPacket p={.seq=wifi_seq[WPK_SRC_SUMMARY]++,.uptime_ms=(uint32_t)(now/1000),.run_id=(uint16_t)runs,
        .result=guard.reason,.flags=wifi_estop?1:0,.elapsed_ms=(uint32_t)(guard.duration_us/1000),
        .distance_mm=(uint32_t)fmin(path_mm(ticks),UINT32_MAX),.line_events=(uint16_t)wifi_line_events,
        .nav_events=(uint16_t)wifi_nav_events,.recoveries=(uint16_t)wifi_recoveries};
    return wifi_forward && wpk_summary(&p,packet) && wifi_send(WIFI_TOPIC_TLM,packet,sizeof(packet));
}
static void wifi_readback(uint8_t id) {
    uint8_t packet[WP_MAP_FRAME_BYTES];wp_map_frame(wifi_mapping,id,wifi_seq[WPK_SRC_MAP]++,packet);
    (void)wifi_send(WIFI_TOPIC_TLM,packet,sizeof(packet));
}
static unsigned wifi_download_row(const WifiCommand *cmd,uint8_t data[64],unsigned *total,unsigned *length) {
    unsigned dataset=cmd->arg>>12,index=cmd->arg&4095;*total=*length=0;
    if(guard.active || trace_settling)return 1;
    if(dataset==WP_LOG_MOTION) {
        *total=trace_count;if(index>=*total)return *total?2:3;
        const BarcodeTrace *b=&trace[(trace_head+B_ROWS-trace_count+index)%B_ROWS];const CircuitTrace *r=&b->motion;
        WifiMotionPacket p={.elapsed_ms=(uint32_t)r->ms,.uptime_ms=(uint32_t)r->ms,.run_id=(uint16_t)runs,
            .phase=(uint8_t)(r->phase&15),.profile=recorded_mode,.flags=(uint8_t)((r->active?1:0)|2|128),.recovery_state=(r->phase&64)?1:0,
            .speed_left_0p1mm_s=r->speed10[0],.speed_right_0p1mm_s=r->speed10[1],
            .target_left_mm_s=r->target[0],.target_right_mm_s=r->target[1],.pwm_left=r->pwm[0],.pwm_right=r->pwm[1],
            .adc_left=r->adc[0],.adc_right=r->adc[1],.adc_barcode=b->side_adc,
            .calibrated_level_left=r->level[0],.calibrated_level_right=r->level[1],.distance_mm=r->path10/10,
            .heading_cdeg=wifi_heading((double)r->wheel10[0]/10,(double)r->wheel10[1]/10),
            .line_levels=(uint8_t)((r->level[0]>=400?1:0)|(r->level[1]>=400?2:0)|(b->ao_black?4:0))};
        if(!wpk_motion(&p,data))return 3;
        *length=WPK_MOTION_BYTES;
    } else if(dataset==WP_LOG_DIAGNOSTIC) {
        *total=diag_capture.count;if(index>=*total)return *total?2:3;
        memcpy(data,diag_capture.rows[(diag_capture.head+DIAG_ROWS-diag_capture.count+index)%DIAG_ROWS].bytes,DIAG_ROW_BYTES);*length=DIAG_ROW_BYTES;
    } else if(dataset==WP_LOG_RAW) {
        *total=raw_capture.header.meta.count;if(index>=*total)return *total?2:3;
        const BarcodeRawRow *r=&raw_capture.rows[index];nav_le32(data,r->us);nav_le32(data+4,(uint32_t)r->path100);
        data[8]=(uint8_t)r->adc;data[9]=(uint8_t)(r->adc>>8);*length=10;
    } else {
        const uint8_t *bank=(const uint8_t *)(XIP_BASE+INTERFACE_LOG_OFFSET);bool restored=!runs;
        *total=restored?(nav_log_valid(bank)?bank[5]:0):motion_log_count();if(index>=*total)return *total?2:3;
        if(restored)memcpy(data,bank+NAV_LOG_HEADER+index*NAV_EVENT_BYTES,NAV_EVENT_BYTES);
        else {MotionEvent e;if(!motion_log_read(index,&e))return 3;nav_event_encode(data,&e);}
        *length=NAV_EVENT_BYTES;
    }
    return 0;
}
static void wifi_pump_download(void) {
    if(!wifi_download)return;
    uint8_t data[64],packet[64],length;unsigned total,n;
    unsigned status=wifi_download_row(&wifi_download_command,data,&total,&n),parts=status?1:(n+48)/49;
    for(unsigned part=0;part<parts;++part) {
        unsigned start=part*49,bytes=status?0:n-start;if(bytes>49)bytes=49;
        if(wpk_log_chunk(wifi_seq[WPK_SRC_LOG_CHUNK]++,(uint8_t)(wifi_download_command.arg>>12),wifi_download_command.id,
            wifi_download_command.arg&4095,(uint16_t)total,(uint8_t)part,(uint8_t)parts,(uint8_t)status,
            data+start,(uint8_t)bytes,packet,&length)) (void)wifi_send(WIFI_TOPIC_TLM,packet,length);
    }
    wifi_download=false;
}
static unsigned wifi_dispatch(const WifiCommand *cmd,uint64_t now) {
    MotionResult result;
    switch(cmd->code) {
    case WP_CMD_START:
        if(wifi_start_pending || start_block(stdio_usb_connected()?LINE_USB_FREE:LINE_BANK))return WP_ACK_BUSY;
        wifi_start_pending=true;wifi_start_at=now+2000000;wifi_stopped=false;return WP_ACK_ACCEPTED;
    case WP_CMD_PAUSE:
        result=nav_request_takeover(MOTION_BUDDY1,&wifi_lease);return result==MOTION_OK?0:WP_ACK_BUSY;
    case WP_CMD_RESUME:
        if(wifi_estop || !wifi_lease)return WP_ACK_BUSY;
        result=nav_resume_line(MOTION_BUDDY1,wifi_lease,false);return result==MOTION_OK?0:WP_ACK_BUSY;
    case WP_CMD_STOP:case WP_CMD_ESTOP:
        wifi_start_pending=false;(void)nav_request_stop(MOTION_BUDDY1);
        {uint32_t irq=save_and_disable_interrupts();kill(now,cmd->code==WP_CMD_ESTOP?WP_ESTOP_REASON:WP_STOP_REASON);restore_interrupts(irq);}
        wifi_estop|=cmd->code==WP_CMD_ESTOP;wifi_stopped=true;return 0;
    case WP_CMD_RESET:
        if(guard.active || trace_settling || !gpio_get(STOP) || !gpio_get(HOLD))return WP_ACK_BUSY;
        wifi_estop=wifi_stopped=false;wifi_start_pending=false;wifi_lease=0;return 0;
    case WP_CMD_CONFIG:wifi_forward=(cmd->arg&4095)!=0;return 0;
    case WP_CMD_PING:wifi_last_hb=0;return 0;
    case WP_CMD_MAP:
        wifi_mapping=wp_map_replace(wifi_mapping,cmd->mapping);nav.mapping=wifi_mapping;
        wifi_readback(cmd->id);wifi_event(now,WPK_EVENT_MAP_APPLIED,0,0,wifi_mapping>>21,0);return 0;
    case WP_CMD_GET:wifi_readback(cmd->id);return 0;
    case WP_CMD_GET_LOG:
        if(guard.active || trace_settling || wifi_download)return WP_ACK_BUSY;
        wifi_download_command=*cmd;wifi_download=true;return 0;
    default:return WP_ACK_UNSUPPORTED;
    }
}
static void wifi_harness_poll(uint64_t now) {
    uint8_t bytes[8];size_t n;
    for(unsigned k=0;k<2 && wifi_transport_receive(bytes,&n);++k) {
        WifiCommand cmd;unsigned result=wp_parse(bytes,n,&cmd);
        wifi_event_id(now,WPK_EVENT_COMMAND_REQUEST,0,0,0,cmd.code,cmd.id);
        if(!result)result=wifi_dispatch(&cmd,now);
        wifi_last_cmd=cmd.code;wifi_last_result=(uint8_t)result;if(result)++wifi_rejected;else ++wifi_accepted;
        uint8_t ack[4];wp_ack(&cmd,(uint8_t)result,wifi_state(),ack);(void)wifi_send(WIFI_TOPIC_ACK,ack,sizeof(ack));
        wifi_event_id(now,WPK_EVENT_COMMAND_RESULT,0,0,0,((unsigned)cmd.code<<8)|result,cmd.id);wifi_last_hb=0;
    }
    if(wifi_start_pending) {
        if(!gpio_get(STOP) || !gpio_get(HOLD) || wifi_estop)wifi_start_pending=false;
        else if(now>=wifi_start_at) {wifi_start_pending=false;if(start_run(stdio_usb_connected()?LINE_USB_FREE:LINE_BANK))recorded_key='m';}
    }
    if(guard.active && !wifi_was_active) {
        wifi_stopped=false;
        wifi_event(now,WPK_EVENT_RUN_BEGIN,0,0,0,0);
    } else if(!guard.active && wifi_was_active) {
        wifi_stopped=true;
        wifi_event(now,WPK_EVENT_STOP,0,0,0,guard.reason);
    }
    wifi_was_active=guard.active;
    WifiTransportStatus transport=wifi_transport_status();
    if(transport.configured && transport.ready && (!wifi_last_hb || now-wifi_last_hb>=1000000)) {
        WifiHeartbeat h={.seq=wifi_seq[WPK_SRC_HEARTBEAT]++,.net=transport.connected&&transport.subscribed?2:1,
            .state=wifi_state(),.last_cmd=wifi_last_cmd,.last_result=wifi_last_result,
            .uptime_s=(uint16_t)fmin(now/1000000,16383),.published=(uint16_t)transport.published,
            .lost=(uint16_t)(transport.tx_dropped+transport.rx_dropped+wifi_app_drops),.accepted=wifi_accepted,.rejected=wifi_rejected,
            .connects=(uint8_t)transport.reconnects,.fill_pct=transport.tx_fill_pct};
        uint8_t packet[WPK_HEARTBEAT_BYTES];if(wpk_heartbeat(&h,packet))(void)wifi_send(WIFI_TOPIC_HB,packet,sizeof(packet));wifi_last_hb=now;
        if(wifi_forward) {
            wifi_event(now,WPK_EVENT_STATUS,0,0,0,0);
            uint32_t irq=save_and_disable_interrupts();
            uint32_t invalid_l=guard.active?enc[0].invalid-initial_invalid[0]:cut_invalid[0];
            uint32_t invalid_r=guard.active?enc[1].invalid-initial_invalid[1]:cut_invalid[1];
            restore_interrupts(irq);
            WifiDiagnosticsPacket diagnostic={.seq=wifi_seq[WPK_SRC_DIAGNOSTICS]++,.uptime_ms=(uint32_t)(now/1000),
                .threshold_adc=(uint16_t)barcode_analog_threshold(side_analog.selected),.bank=side_analog.selected,
                .flags=(uint8_t)((nav.external_scanning?1:0)|(nav.pending?2:0)|(calibration_valid(&calibration)?4:0)|(guard.active?8:0)),
                .decoded_total=nav.decoded_total,.turns=circuit.turns,.lefts=nav.lefts,.rights=nav.rights,.uturns=nav.uturns,
                .bad_frames=nav.rejected,.unsupported=nav.unsupported,.max_adc_gap_us=side_analog.max_gap_us,
                .max_control_gap_us=diag_capture.max_gap,.tx_dropped=transport.tx_dropped,.rx_dropped=transport.rx_dropped,
                .invalid_left=invalid_l,.invalid_right=invalid_r};
            uint8_t diag_packet[WPK_DIAGNOSTICS_BYTES];
            if(wpk_diagnostics(&diagnostic,diag_packet))(void)wifi_send(WIFI_TOPIC_TLM,diag_packet,sizeof(diag_packet));
        }

    }
    if(wifi_forward && now-wifi_last_motion>=50000) {
        WifiMotionPacket m=wifi_snapshot(now);uint8_t packet[WPK_MOTION_BYTES];m.seq=wifi_seq[WPK_SRC_MOTION]++;
        if(wpk_motion(&m,packet))(void)wifi_send(WIFI_TOPIC_TLM,packet,sizeof(packet));
        wifi_line_status(now,WPK_EVENT_STATUS);wifi_last_motion=now;
    }
    if(!guard.active && guard.duration_us && runs && transport.subscribed &&
       (wifi_summary_run!=runs || wifi_summary_connection!=transport.reconnects) && wifi_summary(now)) {
        wifi_summary_run=runs;wifi_summary_connection=transport.reconnects;
    }
    wifi_pump_download();
}
static void wifi_harness_init(void) {
    WifiTransportConfig config;
    if((uintptr_t)&__flash_binary_end<XIP_BASE+WIFI_SETTINGS_OFFSET &&
       wifi_settings_decode(&config,(const uint8_t *)(XIP_BASE+WIFI_SETTINGS_OFFSET))) (void)wifi_transport_configure(&config);
    memset(&config,0,sizeof(config));wifi_transport_init();
    puts("WIFI: n=network status. Settings via provision_wifi.py over USB while stopped; no password is printed. No boot barcode assignments.");
}
static bool wifi_console_byte(int ch) {
    if(ch==PICO_ERROR_TIMEOUT)return false;
    if(!wifi_provision_state && ch=='n') {
        WifiTransportStatus s=wifi_transport_status();
        printf("WIFI: configured=%u stack_ready=%u mqtt=%u subscribed=%u pm_off_verified=%u published=%lu dropped_TX=%lu dropped_RX=%lu poll_gap_us=%lu map_configured=%u map_revision=%u\n",
            s.configured,s.ready,s.connected,s.subscribed,s.pm_disabled,(unsigned long)s.published,(unsigned long)s.tx_dropped,
            (unsigned long)s.rx_dropped,(unsigned long)s.max_poll_gap_us,(wifi_mapping&WP_MAP_CONFIGURED)!=0,(unsigned)(wifi_mapping>>21));
        printf("WIFI_LINK: sdk=%d ip=%u.%u.%u.%u mqtt_attempts=%lu mqtt_result=%d\n",
            (int)s.link_status,(unsigned)(s.ipv4>>24),(unsigned)((s.ipv4>>16)&255u),
            (unsigned)((s.ipv4>>8)&255u),(unsigned)(s.ipv4&255u),
            (unsigned long)s.mqtt_attempts,(int)s.mqtt_result);
        printf("WIFI_PROGRESS: polls=%lu last_ms=%lu phase=%u broker=%u.%u.%u.%u\n",
            (unsigned long)s.polls,(unsigned long)s.last_poll_ms,(unsigned)s.phase,
            (unsigned)(s.broker_ipv4>>24),(unsigned)((s.broker_ipv4>>16)&255u),
            (unsigned)((s.broker_ipv4>>8)&255u),(unsigned)(s.broker_ipv4&255u));
        printf("WIFI_MQTT: stage=%s sdk_state=%u api=%d callback=%d subscribe=%d error=%d init=%d port=1883 age_ms=%lu\n",
            wifi_mqtt_stage_name(s.mqtt_stage),(unsigned)s.mqtt_sdk_state,(int)s.connect_api_result,
            (int)s.mqtt_result,(int)s.subscribe_result,(int)s.last_error,(int)s.init_result,
            (unsigned long)s.stage_age_ms);
        printf("WIFI_SETUP: last_code=%u reason=%s active=%u settling=%u GP20_released=%u GP21_released=%u\n",
            wifi_setup_last_code,wifi_setup_reason(wifi_setup_last_code),guard.active,trace_settling,
            gpio_get(STOP),gpio_get(HOLD));
        return true;
    }
    if(!wifi_provision_state && ch!='!')return false;
    if(!wifi_provision_state){wifi_provision_state=1;wifi_provision_size=0;wifi_start_pending=false;}
    if(ch=='\r')return true;
    if(ch!='\n') {
        if(wifi_provision_size+1<sizeof(wifi_provision_line))wifi_provision_line[wifi_provision_size++]=(char)ch;
        else wifi_provision_state=2;
        return true;
    }
    wifi_provision_line[wifi_provision_size]=0;
    WifiTransportConfig config={.client_id="is28-buddy123"};bool valid=wifi_provision_state==1;
    char *a=wifi_provision_line,*b=strchr(a,'|'),*c=b?strchr(b+1,'|'):NULL,*d=c?strchr(c+1,'|'):NULL;
    if(!b || !c || !d || strchr(d+1,'|'))valid=false;
    if(valid) {
        *b++=*c++=*d++=0;
        valid=!strcmp(a,"!net") && strlen(b)<sizeof(config.ssid) && strlen(c)<sizeof(config.password) && strlen(d)<sizeof(config.broker);
        if(valid){strcpy(config.ssid,b);strcpy(config.password,c);strcpy(config.broker,d);valid=wifi_settings_valid(&config);}
    }
    uint8_t page[256];wifi_setup_last_code=WIFI_SETUP_OK;
    if(!valid)wifi_setup_last_code=WIFI_SETUP_FIELDS;
    else if(guard.active)wifi_setup_last_code=WIFI_SETUP_ACTIVE;
    else if(trace_settling)wifi_setup_last_code=WIFI_SETUP_SETTLING;
    else if(!gpio_get(STOP))wifi_setup_last_code=WIFI_SETUP_STOP_HELD;
    else if(!gpio_get(HOLD))wifi_setup_last_code=WIFI_SETUP_HOLD_HELD;
    else if((uintptr_t)&__flash_binary_end>=XIP_BASE+WIFI_SETTINGS_OFFSET)wifi_setup_last_code=WIFI_SETUP_OVERLAP;
    else if(!wifi_settings_encode(page,&config))wifi_setup_last_code=WIFI_SETUP_ENCODE;
    else if(!wifi_transport_flash_begin())wifi_setup_last_code=WIFI_SETUP_LOCKOUT;
    else {
        outputs_off();uint32_t irq=save_and_disable_interrupts();flash_range_erase(WIFI_SETTINGS_OFFSET,FLASH_SECTOR_SIZE);
        flash_range_program(WIFI_SETTINGS_OFFSET,page,sizeof(page));restore_interrupts(irq);wifi_transport_flash_end();
        if(memcmp((const uint8_t *)(XIP_BASE+WIFI_SETTINGS_OFFSET),page,sizeof(page)))wifi_setup_last_code=WIFI_SETUP_VERIFY;
    }
    memset(&config,0,sizeof(config));memset(page,0,sizeof(page));memset(wifi_provision_line,0,sizeof(wifi_provision_line));
    wifi_provision_state=0;wifi_provision_size=0;
    if(wifi_setup_last_code==WIFI_SETUP_OK) {
        puts("WIFI_PROVISIONED: verified; rebooting. Password hidden.");
        stdio_flush();sleep_ms(100);watchdog_reboot(0,0,0);
    } else printf("WIFI_SETUP_REJECTED: code=%u reason=%s\n",wifi_setup_last_code,wifi_setup_reason(wifi_setup_last_code));
    return true;
}
