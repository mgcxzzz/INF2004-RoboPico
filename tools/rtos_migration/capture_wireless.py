"""Passive MQTT capture for the network-supervised main RTOS release. No car commands."""
from __future__ import annotations
import argparse,csv,datetime as dt,json,struct,threading,time
from pathlib import Path
from capture_connected_line import monitor
from capture_network_bench import heartbeat

KEYS=('uptime_ms','reads','failed','late_gaps','fifo_overruns','fifo_budget','init_budget',
      'control_count','control_late','holds','recoveries','disarms','early_clears','epoch')
def decode_release(frame):
    if len(frame)<3 or frame[0]!=0xf2 or len(frame)!=3+frame[1]:return None
    p=frame[3:]
    if len(p)==12 and p[0]==0xc2 and not p[2]&128 and not p[3]&224:
        packed=int.from_bytes(p[4:10],'little');levels=[(packed>>(12*k))&4095 for k in range(4)]
        return {'kind':'release','version':f'{p[1]&15}.{p[1]>>4}.{p[2]&15}',
          'network_supervised':bool(p[2]&16),'stored_logs':bool(p[2]&32),
          'calibration_valid':bool(p[2]&64),'profile':p[3]&3,'origin':(p[3]>>2)&3,
          'recorded_calibration':bool(p[3]&16),'white':levels[:2],'black':levels[2:],
          'run_id':int.from_bytes(p[10:12],'little')}
    if len(p)==60 and p[0]==0xc3 and not p[58]&192 and not p[59]&128:
        return dict(kind='runtime',**dict(zip(KEYS,struct.unpack('<14I',p[1:57]))),
          clear_reason=p[57]&15,failure_reason=p[57]>>4,hold_reason=p[58]&3,
          qualification_flags=(p[58]>>2)&15,control_flags=p[59])
    if len(p)==45 and p[0]==0xc4 and p[1]==0x21 and not p[2]&192:
        flags=p[2]; valid=bool(flags&1); kind=(flags>>1)&3;wheel=(flags>>3)&3
        if not valid:
            return {'kind':'encoder-fault','valid':False} if not flags and not any(p[3:]) else None
        if not kind or wheel not in (1,2) or p[13]&254 or p[37]&252 or p[40]&192:return None
        direction=int.from_bytes(p[11:14],'little');quality=int.from_bytes(p[14:19],'little')
        target=int.from_bytes(p[35:38],'little');pwm=int.from_bytes(p[38:41],'little')
        signed=lambda x,n:x-(1<<n) if x&(1<<(n-1)) else x
        targets=[signed((target>>(9*k))&511,9) for k in range(2)]
        pwms=[signed((pwm>>(11*k))&2047,11) for k in range(2)]
        if any(abs(v)>180 for v in targets) or any(abs(v)>1000 for v in pwms) or (kind!=3 and direction):return None
        return {'kind':'encoder-fault','valid':True,'trip_kind':kind,'wheel':wheel,'clamped':bool(flags&32),
            'timer_us':int.from_bytes(p[3:7],'little'),'kernel_ms':int.from_bytes(p[7:11],'little'),
            'direction_us':direction,'window_bad':quality&0xfffff,'window_changes':quality>>20,
            'lifetime_edges':list(struct.unpack_from('<I',p,19))+list(struct.unpack_from('<I',p,27)),
            'lifetime_invalid':list(struct.unpack_from('<I',p,23))+list(struct.unpack_from('<I',p,31)),
            'targets':targets,'pwm':pwms,'speed_x10':list(struct.unpack_from('<2h',p,41))}
    if len(p)==27 and p[0]==0xc5:
        meta=int.from_bytes(p[23:27],'little');flags=meta&63;code=(meta>>6)&31;reason=(meta>>11)&15
        bars=(meta>>15)&31;count=meta>>20
        if code>26 or reason>10 or bars>24 or count>2048:return None
        if bool(flags&2)!=bool(code) or (flags&32 and not flags&2):return None
        if not flags&1 and (meta or any(p[5:21])):return None
        if reason==10 and flags&4:return None
        return {'kind':'barcode-attempt','uptime_ms':int.from_bytes(p[1:5],'little'),
            'attempt':int.from_bytes(p[5:9],'little'),'first_ms':int.from_bytes(p[9:13],'little'),
            'last_ms':int.from_bytes(p[13:17],'little'),'overwritten':int.from_bytes(p[17:21],'little'),
            'run_id':int.from_bytes(p[21:23],'little'),'valid':bool(flags&1),'decoded':bool(flags&2),
            'finalized':bool(flags&4),'truncated':bool(flags&8),'start_valid':bool(flags&16),
            'original_accepted':bool(flags&32),'code':chr(code+64) if code else '?',
            'reason':reason,'bars':bars,'raw_count':count}
    if len(p)==29 and p[0]==0xc6 and p[1]==0x21 and p[2]==0:
        meta=int.from_bytes(p[27:29],'little')
        if meta&0xc000:return None
        valid=bool(meta&8192);link=(meta>>3)&7;link=link-8 if link&4 else link
        if not valid:return {'kind':'network-stop','valid':False} if not meta and not any(p[3:27]) else None
        fail=(meta>>6)&127
        if not fail or link<-3:return None
        values=struct.unpack_from('<6I',p,3)
        return dict(kind='network-stop',valid=True,flags=meta&7,link=link,fail_mask=fail,
            **dict(zip(('stop_ms','poll_age_ms','app_age_ms','ack_age_ms','connects','last_complete_ms'),values)))
    return None

def summarize(frames,interruptions):
    rows=[];releases=[];beats=[];diagnostics=[]
    for f in frames:
        if f.get('retained'):continue
        p=bytes.fromhex(f['hex'])
        if f['topic']=='is28/hb' and heartbeat(p):beats.append(f['wall_s'])
        if f['topic']!='is28/tlm':continue
        d=decode_release(p)
        if d:
            bucket=rows if d['kind']=='runtime' else releases if d['kind']=='release' else diagnostics
            bucket.append(dict(wall_s=f['wall_s'],**d))
    windows=[]
    for epoch in dict.fromkeys(r['epoch'] for r in rows):
        group=[r for r in rows if r['epoch']==epoch];a,b=group[0],group[-1]
        wall=b['wall_s']-a['wall_s'];delta={k:(b[k]-a[k])&0xffffffff for k in KEYS if k not in ('epoch','uptime_ms')}
        windows.append({'epoch':epoch,'first_wall_s':a['wall_s'],'last_wall_s':b['wall_s'],
            'wall_seconds':wall,'car_elapsed_ms':(b['uptime_ms']-a['uptime_ms'])&0xffffffff,
            'delta':delta,'imu_hz_wall':delta['reads']/wall if wall>0 else None,
            'control_hz_wall':delta['control_count']/wall if wall>0 else None})
    return rows,{'result':'REVIEW_PHYSICAL_TEST','frames':len(frames),'heartbeats':len(beats),
      'max_heartbeat_gap_s':max((b-a for a,b in zip(beats,beats[1:])),default=None),
      'release_first':releases[0] if releases else None,'release_last':releases[-1] if releases else None,
      'runtime_first':rows[0] if rows else None,'runtime_last':rows[-1] if rows else None,
      'runtime_windows':windows,'monitor_interruptions':interruptions,
      'diagnostic_latest':{k:next((d for d in reversed(diagnostics) if d['kind']==k),None) for k in ('encoder-fault','barcode-attempt','network-stop')},
      'scope':'Passive laptop receipt evidence. Closing this observer does not stop the robot. Stop first; download stored rows before a new Start or power cycle. No automatic physical or clock certification.'}

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--broker',default='127.0.0.1')
    p.add_argument('--seconds',type=float,default=300);p.add_argument('--output',type=Path,default=Path('rtos_wireless_session'))
    a=p.parse_args()
    if a.seconds<10:p.error('Capture at least 10 seconds.')
    a.output.mkdir(parents=True,exist_ok=True);prefix=a.output/dt.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    frames=[];interruptions=[];stop=threading.Event();started=time.monotonic()
    worker=threading.Thread(target=monitor,args=(a.broker,stop,frames,interruptions,started),daemon=True);worker.start()
    print('Wireless MQTT capture: no USB/COM port and no robot commands. Keep broker/dashboard running.')
    print('Use dashboard Start/Stop. STOP THE ROBOT BEFORE Ctrl+C. Closing capture does not stop motion.')
    try:
        while time.monotonic()-started<a.seconds:stop.wait(.2)
    except KeyboardInterrupt:pass
    finally:stop.set();worker.join(7)
    runtime,result=summarize(frames,interruptions)
    with prefix.with_name(prefix.name+'_mqtt.csv').open('w',newline='',encoding='utf-8') as f:
        w=csv.DictWriter(f,fieldnames=['wall_s','topic','hex','retained']);w.writeheader();w.writerows(frames)
    if runtime:
        with prefix.with_name(prefix.name+'_runtime.csv').open('w',newline='',encoding='utf-8') as f:
            w=csv.DictWriter(f,fieldnames=list(runtime[0]));w.writeheader();w.writerows(runtime)
    prefix.with_name(prefix.name+'_summary.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print('Capture saved to',a.output.resolve());print(json.dumps(result,indent=2))
if __name__=='__main__':main()
