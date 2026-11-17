/* Stopped-only front IR wizard. Car readback, rather than ACK, confirms changes. */
(function(root) {
  'use strict';
  function create({send, connected, now = () => Date.now()}) {
    const el=id=>document.getElementById(id);
    let status=null, seen=0, pending=null, notice='Waiting for calibration status. Firmware v0.2.8+ is required.';
    const results=['Ready','Session open','Capturing — hold still','Capture complete','Applied in RAM',
      'Saved and verified','Pending samples discarded','Invalid or incomplete calibration','Busy — Stop first',
      'ADC unavailable','Capture unstable — hold still and retry','Session expired','Flash verification failed — active settings retained'];
    const actions={1:'Begin calibration',2:'Capture both white',3:'Capture left black',4:'Capture right black',
      5:'Use in RAM',6:'Save and use',7:'Discard pending samples'};
    function refresh() {
      const fresh=!!status && connected() && now()-seen<2500;
      if (pending && now()-pending.at>6000) {
        notice='Result not confirmed. Read status before retrying; do not assume the setting changed.';pending=null;
      }
      el('calStatus').textContent=fresh ? results[status.result] : 'No fresh calibration status — changes disabled';
      el('calNotice').textContent=notice;
      el('calRaw').textContent=fresh && status.adcReady ? `${status.raw[0]} / ${status.raw[1]} ADC` : 'Unavailable';
      el('calActive').textContent=fresh ? `White ${status.activeWhite.join(' / ')} · black ${status.activeBlack.join(' / ')}` : 'Unknown';
      const staged= fresh ? status.stagedWhite.concat(status.stagedBlack) : [];
      ['WhiteL','WhiteR','BlackL','BlackR'].forEach((suffix,index)=>{
        const bit=index<2 ? 1 : index===2 ? 2 : 4;
        el('cal'+suffix).textContent=fresh && (status.captured&bit) ? String(staged[index]) : '—';
      });
      el('calProgress').textContent=fresh && status.capturing ? `${status.samples} / 32 fresh samples` : '';
      el('calSpan').textContent=fresh && status.captured===7 ?
        `Span L/R ${status.stagedBlack[0]-status.stagedWhite[0]} / ${status.stagedBlack[1]-status.stagedWhite[1]} ADC · ${status.valid?'valid':'invalid — recapture before applying'}` : 'Capture all three positions before applying.';
      for (const button of document.querySelectorAll('[data-cal-op]')) {
        const op=Number(button.dataset.calOp);
        let allowed=fresh && status.canChange && status.adcReady && !pending && !status.capturing;
        if (op>=2 && op<=4) allowed=allowed && status.session && (op===2 || !!(status.captured&1));
        if (op===5 || op===6) allowed=allowed && status.valid && (status.session || status.applied);
        if (op===7) allowed=fresh && status.session; // Cancel may interrupt capture.
        button.disabled=!allowed;
      }
      el('calRead').disabled=!connected();
    }
    for (const button of document.querySelectorAll('[data-cal-op]')) {
      button.addEventListener('click',()=>{
        refresh();if(button.disabled)return;
        const operation=Number(button.dataset.calOp),id=send(operation);
        if (id===null || id===undefined)return;
        pending={id,operation,at:now()};notice=`${actions[operation]} requested. Waiting for the car's result…`;refresh();
      });
    }
    el('calRead').addEventListener('click',()=>{send(0);notice='Read requested; waiting for fresh status.';refresh();});
    refresh();const timer=setInterval(refresh,500);
    return {
      update(value) {
        status=value;seen=now();
        if(pending && value.requestId===pending.id && value.operation===pending.operation) {
          if(value.capturing) notice='Capturing 32 fresh readings. Keep the robot flat and still.';
          else {notice=results[value.result];pending=null;}
        }
        refresh();
      },
      ack(value,own) {
        if(!own || !pending || value.id!==pending.id || value.code!==7)return;
        if(value.result!==0) {notice=`Request rejected (${value.result}). Active settings were not confirmed changed.`;pending=null;}
        else notice='Request accepted; waiting for calibration readback. Acceptance alone does not confirm application or saving.';
        refresh();
      },
      invalidate(reason) {status=null;seen=0;pending=null;notice=reason;refresh();},
      destroy() {clearInterval(timer);}
    };
  }
  root.RoboPicoCalibration={create};
})(typeof window!=='undefined'?window:globalThis);
