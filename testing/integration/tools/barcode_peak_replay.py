#!/usr/bin/env python3
"""Offline peak-based Code39 replay of raw barcode ADC captures (host only).
Bars = local maxima of the ADC trace (wide if > WIDE_ADC); spaces = bar-centre
spacing minus nominal half-widths (3/9 mm), wide if > 6 mm. Accepts a frame only
if all 9-element groups decode and it starts/ends with '*'.
Usage: barcode_peak_replay.py CAPTURE.csv [...]   (index,us,distance_mm,adc)
Diagnostic prototype, not firmware."""
import csv,sys,numpy as np
from scipy.signal import find_peaks
T={'0':"nnnwwnwnn",'1':"wnnwnnnnw",'2':"nnwwnnnnw",'3':"wnwwnnnnn",'4':"nnnwwnnnw",'5':"wnnwwnnnn",'6':"nnwwwnnnn",'7':"nnnwnnwnw",'8':"wnnwnnwnn",'9':"nnwwnnwnn",
'A':"wnnnnwnnw",'B':"nnwnnwnnw",'C':"wnwnnwnnn",'D':"nnnnwwnnw",'E':"wnnnwwnnn",'F':"nnwnwwnnn",'G':"nnnnnwwnw",'H':"wnnnnwwnn",'I':"nnwnnwwnn",'J':"nnnnwwwnn",
'K':"wnnnnnnww",'L':"nnwnnnnww",'M':"wnwnnnnwn",'N':"nnnnwnnww",'O':"wnnnwnnwn",'P':"nnwnwnnwn",'Q':"nnnnnnwww",'R':"wnnnnnwwn",'S':"nnwnnnwwn",'T':"nnnnwnwwn",
'U':"wwnnnnnnw",'V':"nwwnnnnnw",'W':"wwwnnnnnn",'X':"nwnnwnnnw",'Y':"wwnnwnnnn",'Z':"nwwnwnnnn",'*':"nwnnwnwnn"}
R={v:k for k,v in T.items()}
def load(f):
    r=list(csv.DictReader(open(f))); a=np.array([int(x['adc']) for x in r],float); d=np.array([float(x['distance_mm']) for x in r]); return d,a
def dec(d,a,prom=5,smooth=5,wideadc=600):
    g=np.arange(d[0],d[-1],0.1); idx=np.searchsorted(d,g)
    s=np.array([a[max(i-1,0):i+2].mean() for i in idx]); s=np.convolve(s,np.ones(smooth)/smooth,'same')
    base=np.percentile(s,10)
    pk,pr=find_peaks(s,prominence=prom,distance=15,height=base+25)
    bars=[(g[p],'w' if s[p]>wideadc else 'n',s[p]) for p in pk]
    # widths: wide bar width = span above half its height; spaces from centre spacing
    if len(bars)<15: return bars,None,'only %d bars'%len(bars)
    # take the 15 bars with highest-confidence: assume exactly one barcode: use all
    els=[]
    for i,b in enumerate(bars):
        els.append(b[1])
        if i+1<len(bars):
            c=bars[i+1][0]-b[0]; bw=lambda t:9 if t=='w' else 3
            sp=c-(bw(b[1])+bw(bars[i+1][1]))/2
            els.append(('w' if sp>6 else 'n',round(sp,1)))
    pat=''.join(e if isinstance(e,str) else e[0] for e in els)
    out=[]
    for direction,p in (('fwd',pat),('rev',pat[::-1])):
        chars=[];ok=True
        for k in range(0,len(p),10):
            c=p[k:k+9]
            if len(c)<9: break
            chars.append(R.get(c,'?'))
            if k+9<len(p) and p[k+9]!='n': chars.append('|gapW')
        out.append((direction,''.join(chars)))
    return bars,els,out
for f in sys.argv[1:]:
    d,a=load(f); bars,els,out=dec(d,a)
    print('##',f.split('/')[-1],len(bars),'bars:',' '.join('%.1f%s'%(b[0],b[1]) for b in bars))
    if els: print('   spaces:',[e[1] for e in els if not isinstance(e,str)]); print('  ',out)
