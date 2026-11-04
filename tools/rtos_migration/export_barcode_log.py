"""Export stopped Step4 barcode events and 2ms raw GP27 history. Sends b only."""
from __future__ import annotations
import argparse,csv,datetime as dt,json,re,time
from pathlib import Path
EVENTS='sequence,ms,path_x10mm,width_x100mm,event,letter_ascii,action_ascii,map_revision,flags,pattern_start,pattern_letter,pattern_stop'.split(',')
RAW='index,ms,distance_x100mm,adc'.split(',')

def parse_capture(lines):
    events=[];raw=[];issues=[];header=None;end=None;cal=None;started=False;finished=False
    for line in lines:
        if line.startswith('BARCODE_LOG_BEGIN '):
            m=re.fullmatch(r'BARCODE_LOG_BEGIN version=1 events=(\d+) raw=(\d+) overwritten=(\d+) frozen=([01])',line)
            if not m or started:issues.append('Invalid/duplicate barcode header.');continue
            header=list(map(int,m.groups()));started=True
        elif line.startswith('BARCODE_LOG_CAL ') and started:cal=line
        elif started and not finished and (line.startswith('BARCODE_EVENT ') or line.startswith('BARCODE_RAW ')):
            event=line.startswith('BARCODE_EVENT ');columns=EVENTS if event else RAW
            try:
                values=list(map(int,line.split(' ',1)[1].split(',')))
                if len(values)!=len(columns):raise ValueError('column count')
                row=dict(zip(columns,values));collection=events if event else raw
                if event:
                    if any(not 0<=row[k]<=255 for k in ['event','letter_ascii','action_ascii','map_revision','flags']):raise ValueError('byte field')
                    if any(not 0<=row[k]<=511 for k in ['pattern_start','pattern_letter','pattern_stop']):raise ValueError('Code39 pattern')
                    if collection and row['sequence']!=collection[-1]['sequence']+1:raise ValueError('event sequence')
                else:
                    if row['index']!=len(raw):raise ValueError('raw index')
                    if not 0<=row['adc']<=4095:raise ValueError('ADC range')
                    if not -2147483648<=row['distance_x100mm']<=2147483647:raise ValueError('distance range')
                if row['ms']<0 or (collection and row['ms']<collection[-1]['ms']):raise ValueError('time order')
                collection.append(row)
            except ValueError as e:issues.append('Malformed barcode row: '+str(e))
        elif started and line.startswith('BARCODE_LOG_END '):
            m=re.fullmatch(r'BARCODE_LOG_END events=(\d+) raw=(\d+)',line)
            if m:end=list(map(int,m.groups()));finished=True
    if header is None or not finished:issues.append('Incomplete barcode dump; stop and use Step4 firmware.')
    if header and (header[:2]!=[len(events),len(raw)] or end!=header[:2]):issues.append('Row counts disagree with header/end.')
    if cal is None:issues.append('Run calibration metadata missing.')
    gaps=[b['ms']-a['ms'] for a,b in zip(raw,raw[1:])]
    summary={'result':'INCOMPLETE' if issues else 'COMPLETE','issues':issues,'events':len(events),
        'raw_samples':len(raw),'raw_overwritten':header[2] if header else None,
        'raw_frozen_on_decode':bool(header[3]) if header else None,'run_calibration':cal,
        'decoded':[e for e in events if e['event']==3],
        'scope':'Export integrity only. Raw is a bounded ring, frozen on the first accepted decode; event/physical acceptance still requires review.'}
    if raw:summary.update(raw_first_ms=raw[0]['ms'],raw_last_ms=raw[-1]['ms'],
        adc_min=min(r['adc'] for r in raw),adc_max=max(r['adc'] for r in raw),max_raw_gap_ms=max(gaps,default=0))
    return events,raw,summary

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--port');p.add_argument('--input',type=Path)
    p.add_argument('--output',type=Path,default=Path('rtos_barcode_session'));a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=True);prefix=a.output/dt.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    if a.input:lines=a.input.read_text(encoding='utf-8-sig').splitlines()
    else:
        try:
            import serial
            import serial.tools.list_ports
        except ImportError:raise SystemExit('Install pyserial once: py -3 -m pip install pyserial')
        if not a.port:
            for item in serial.tools.list_ports.comports():print(item.device,item.description)
            a.port=input('Pico USB port: ').strip()
        if a.port.isdecimal():a.port='COM'+a.port
        print('Robot must be stopped. Close PuTTY/capture; KEEP Pico power connected. Sending b only.')
        lines=[]
        try:
            with serial.Serial(a.port,115200,timeout=.25,write_timeout=2) as port:
                time.sleep(.5);port.write(b'b');deadline=time.monotonic()+120
                while time.monotonic()<deadline:
                    line=port.readline().decode('utf-8',errors='replace').strip()
                    if not line:continue
                    lines.append(line)
                    if not line.startswith(('BARCODE_EVENT ','BARCODE_RAW ')):print(line)
                    if line.startswith(('BARCODE_LOG_END ','LOG rejected:')):break
        except serial.SerialException as error:print('Export interrupted; retaining partial evidence:',error)
    prefix.with_name(prefix.name+'_usb.txt').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    events,raw,summary=parse_capture(lines)
    for suffix,columns,data in [('events',EVENTS,events),('raw',RAW,raw)]:
        with prefix.with_name(prefix.name+'_'+suffix+'.csv').open('w',encoding='utf-8',newline='') as f:
            w=csv.DictWriter(f,fieldnames=columns);w.writeheader();w.writerows(data)
    prefix.with_name(prefix.name+'_summary.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(summary,indent=2));print('Send all FOUR files from',a.output.resolve())
    return 0 if summary['result']=='COMPLETE' else 2
if __name__=='__main__':raise SystemExit(main())
