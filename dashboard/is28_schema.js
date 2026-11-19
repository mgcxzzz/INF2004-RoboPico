/*
 * is28_schema.js (merged dashboard v0.1) -- decode IS28 telemetry from BOTH
 * firmware families with the decoders they already ship:
 *
 *   RTOS (micro T-Kernel, tools/dashboard/is28_schema.js, sys_tlm.h):
 *     sources 1-8 (WiFi status, Buddy2 motion/side geometry, Buddy3 line,
 *     Buddy4 IMU, Buddy5 obstacle, navigation, safety, barcode map).
 *   SDK v2.3 (testing/subsystem2/interface_wifi_v2_3/dashboard/is28_schema.js,
 *     wifi_packets.h/.c): sources 1, 8, 9-14 (motion, line/barcode events,
 *     coordinator events, run summary, variable-length log chunks,
 *     diagnostics). The v2.3 + Buddy4 add-on also emits sources 2 and 4 with
 *     the RTOS layouts (unchanged frames).
 *
 * Every function body below is copied from one of those two files; only
 * names that collided were made distinct (RTOS source-2 decodeMotion ->
 * decodeB2Motion; v2.3 source-9 decodeMotion keeps its name). No field,
 * width, sign, unit or validity rule was changed. Frames are never
 * truncated: SDK packets may be up to 64 bytes (WIFI_PACKET_MAX), RTOS up to
 * 40 bytes (B1_OUT_MAX); each decoder checks its own exact length.
 *
 * Header: [source << 4 | version 2, payload length, sequence], then
 * little-endian fields.
 */
(function (root) {
  'use strict';

  const VERSION = 2;
  const SRC = { WIFI: 1, MOTION: 2, LINE: 3, IMU: 4, OBSTACLE: 5, NAV: 6, SAFETY: 7,
                BARCODE_MAP: 8, SDK_MOTION: 9, SDK_LINE: 10, NAV_EVENT: 11, SUMMARY: 12,
                LOG_CHUNK: 13, DIAGNOSTICS: 14, SOURCE15: 15 };
  const SRC_NAME = ['?', 'WiFi (B1)', 'Motion (B2)', 'Line (B3)', 'IMU (B4)',
                    'Obstacle (B5)', 'Navigation', 'Safety', 'Barcode mapping',
                    'Motion telemetry', 'Line/barcode event', 'Coordinator event',
                    'Run summary', 'Log data chunk', 'Diagnostics status',
                    'Source-15 diagnostics'];
  /* Which firmware emits each source. 'shared' = both families with the
   * same layout; 'rtos'/'sdk' = only that family. Sources 2/4 are RTOS
   * Buddy2/Buddy4 frames, also emitted by the v2.3 + Buddy4 add-on. */
  const SRC_FAMILY = { 1: 'shared', 2: 'shared', 3: 'rtos', 4: 'shared', 5: 'rtos', 6: 'rtos',
                       7: 'rtos', 8: 'shared', 9: 'sdk', 10: 'sdk', 11: 'sdk', 12: 'sdk',
                       13: 'sdk', 14: 'sdk', 15: 'sdk' };
  const MAX_FRAME = { rtos: 40, sdk: 64 };

  const NAV_STATE = ['Ready', 'Running', 'Paused', 'Safe Stop', 'Completed',
                     'Emergency Stop', 'Fault', 'unknown'];
  const NAV_OWNER = ['none', 'line', 'barcode', 'search', 'obstacle'];
  const SAFETY_STATE = ['Normal', 'Emergency Stop', 'Fault'];
  const SAFETY_SRC = ['WiFi', 'Motion', 'Line', 'IMU', 'Obstacle', 'Nav', 'Safety', 'none'];
  const FAULT_BITS = ['motor stall', 'encoder', 'IMU', 'no echo', 'WiFi/broker lost',
                      'servo', 'line sensor', 'obstacle'];
  const NET_STATE = ['Disconnected', 'Connecting', 'Operational', 'Reconnect wait'];
  const B5_STATE = ['Idle', 'Guard', 'Confirm', 'Halting', 'Coarse sweep', 'Edge refine',
                    'Planning', 'Bypassing', 'Recovering', 'Blocked', 'Fault'];
  const B5_FAULT = ['none', 'echo timeout', 'servo', 'line lost', 'blocked',
                    'stop timeout', 'motion'];
  const B5_SIDE = ['none', 'right', 'left'];
  const S3_STATE = ['Initialising', 'Idle', 'Line following', 'Read barcode',
                    'Await navigation', 'Fault'];
  const S3_RESULT = ['ok', 'bad length', 'bad ratio', 'bad character', 'no start/stop',
                     'backwards', 'timeout/overflow'];
  const S3_CMD = ['none', 'left', 'right', 'straight', 'U-turn'];
  const CODE39 = '0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%*';
  const IMU_STATE = ['Initialising', 'Calibrating', 'Idle', 'Tracking', 'Hump',
                     'Impact', 'Fault'];
  const IMU_MOTION = ['unknown', 'stationary', 'steady', 'accelerating', 'turning',
                      'climbing', 'descending', 'impact'];
  const IMU_EVT = ['status', 'motion', 'hump start', 'hump peak', 'hump rejected',
                   'impact', 'fault', 'run summary'];
  /* SDK v2.3 coordinator (source 11) names, from the v2.3 dashboard. */
  const SDK_MODE = ['Auto', 'Stopping', 'Paused', 'Driving', 'Turning', 'Settling', 'Fault', 'Off'];
  const SDK_OWNER = ['none', 'Buddy 1', 'Buddy 2', 'Buddy 3', 'Buddy 4', 'Buddy 5'];
  const SDK_MODE_OFF = 7, SDK_MODE_FAULT = 6;

  /* Commands. 11 = GET_LOG exists in SDK v2.3 only; RTOS answers it as an
   * unknown command. 12-14 are prohibited direct-drive codes (FR-WIFI-03). */
  const CMD = { START: 1, PAUSE: 2, RESUME: 3, STOP: 4, RESET: 5, ESTOP: 6, CONFIG: 7,
                PING: 8, BARCODE_MAP: 9, BARCODE_GET: 10, GET_LOG: 11,
                STEER: 12, DRIVE: 13, TURN: 14 };
  const CMD_NAME = ['none', 'START', 'PAUSE', 'RESUME', 'STOP', 'RESET', 'ESTOP',
                    'CONFIG', 'PING', 'BARCODE_MAP', 'BARCODE_GET', 'GET_LOG', 'STEER', 'DRIVE',
                    'TURN', 'code15'];
  const MOTION_CMDS = [1, 2, 3, 4, 5, 6];   /* commands that can change car motion state */
  const ACK_NAME = ['accepted', 'rejected: steering/navigation not allowed (FR-WIFI-03)',
                    'rejected: unknown command', 'rejected: bad format',
                    'rejected: queue full', 'rejected: unsupported setting'];
  const CFG_TELEMETRY = 1;
  const CFG_DRIVE_MODE = 2;
  const CFG_CALIBRATION = 3;
  const DRIVE_MODES = ['Baseline · 45 mm/s', 'Normal · 60 mm/s', 'Faster trial · 75 mm/s'];

  const EVENT_NAME = ['status','run_begin','decoded','queued','junction_confirmed','dispatch','completed',
                      'recovery_wait','recovery_sweep','recovery_found','stop','map_applied',
                      'command_request','command_result','fault','unassigned'];
  const PROFILE_NAME = ['unknown','normal 45','ramp 180 experiment','qualified 135'];
  const LOG_DATASETS = [
    { name: 'motion', label: 'Motion rows', rowBytes: 48 },
    { name: 'diagnostics', label: 'Diagnostics', rowBytes: 58 },
    { name: 'raw-sensors', label: 'Raw sensor rows', rowBytes: 10 },
    { name: 'navigation', label: 'Navigation rows', rowBytes: 31 }
  ];

  /* Fixed payload length per source (source 2 also 24/32/36/37; source 13
   * is variable and validated by decodeLogChunk). */
  const PAYLOAD = { 1: 12, 2: 20, 3: 12, 4: 16, 5: 16, 6: 6, 7: 12, 8: 5,
                    9: 45, 10: 21, 11: 23, 12: 22, 14: 60 };
  const B2_LENGTHS = [20, 24, 32, 36, 37];
  const MAP_MOVEMENTS = ['Forward', 'U-turn', 'Left', 'Right'];

  /* ---------- shared helpers ---------- */
  function u(word, shift, bits) {
    return bits >= 32 ? (word >>> shift) >>> 0 : (word >>> shift) & ((1 << bits) - 1);
  }
  function s(word, shift, bits) {
    const raw = u(word, shift, bits);
    if (bits === 32) return raw | 0;
    const sign = 1 << (bits - 1);
    return (raw ^ sign) - sign;
  }
  function le32(b, i) {
    return (b[i] | (b[i + 1] << 8) | (b[i + 2] << 16) | (b[i + 3] << 24)) >>> 0;
  }
  function i16(p, i) { const n = p[i] | (p[i + 1] << 8); return n & 0x8000 ? n - 0x10000 : n; }
  function u16(p, i) { return p[i] | (p[i + 1] << 8); }
  function u32(p, i) { return le32(p, i); }
  function ascii(p, i) { return p[i] ? String.fromCharCode(p[i]) : '-'; }
  function name(list, i) { return i < list.length ? list[i] : ('#' + i); }

  /* ---------- barcode mapping (identical in both) ---------- */
  function mappingWord(letters) {
    if (!Array.isArray(letters) || letters.length !== 4 ||
        letters.some((x) => typeof x !== 'string' || !/^[A-Z]$/.test(x)) ||
        new Set(letters).size !== 4) {
      throw new Error('Choose four different letters from A to Z.');
    }
    return letters.reduce((word, ch, i) => word | ((ch.charCodeAt(0) - 65) << (5 * i)), 0);
  }
  function encodeMapping(id, letters) {
    const word = mappingWord(letters);
    return new Uint8Array([(CMD.BARCODE_MAP << 4) | VERSION, id & 255,
                           word & 255, (word >>> 8) & 255, (word >>> 16) & 255]);
  }
  function decodeMapping(p) {
    const word = le32(p, 0), configured = !!(word & (1 << 20));
    if ((word >>> 29) || (!configured && word !== 0)) return { error: 'invalid mapping status' };
    const indices = [0, 1, 2, 3].map((i) => (word >>> (5 * i)) & 31);
    if (configured && (indices.some((i) => i >= 26) || new Set(indices).size !== 4)) {
      return { error: 'invalid mapping letters' };
    }
    return { configured, revision: (word >>> 21) & 255, requestId: p[4],
             letters: configured ? indices.map((i) => String.fromCharCode(65 + i)) : [] };
  }
  function mappedMovement(mapping, letter) {
    if (!mapping || !mapping.configured) return 'not configured';
    const slot = mapping.letters.indexOf(letter);
    return slot >= 0 ? MAP_MOVEMENTS[slot] : 'unassigned';
  }
  function mappingReadbackStatus(mapping, wanted) {
    if (!mapping || mapping.error) return 'invalid';
    if (!mapping.configured) return wanted ? 'mismatch' : 'not_configured';
    if (wanted && (!Array.isArray(wanted) || wanted.length !== 4 ||
        !mapping.letters || !mapping.letters.every((ch,i)=>ch===wanted[i]))) return 'mismatch';
    return 'confirmed';
  }
  function mappingTimedOut(pending, timestamp, timeoutMs) {
    return !!pending && timestamp - pending.time >= (timeoutMs === undefined ? 5000 : timeoutMs);
  }

  /* ---------- source 1 (shared Buddy1 v2 status) ---------- */
  function decodeB1(p) {
    const w0 = le32(p, 0), w1 = le32(p, 4), w2 = le32(p, 8);
    const result = u(w0, 9, 3);
    return {
      net: u(w0, 0, 2), netName: name(NET_STATE, u(w0, 0, 2)),
      state: u(w0, 2, 3), stateName: name(NAV_STATE, u(w0, 2, 3)),
      lastCmd: u(w0, 5, 4), lastCmdName: CMD_NAME[u(w0, 5, 4)],
      lastResult: result, lastResultName: result === 7 ? 'none yet' : name(ACK_NAME, result),
      uptimeS: u(w0, 12, 14),
      published: u(w1, 0, 16), lost: u(w1, 16, 16),
      accepted: u(w2, 0, 8), rejected: u(w2, 8, 8), connects: u(w2, 16, 8),
      fillPct: u(w2, 24, 7)
    };
  }

  /* ---------- RTOS sources 2-7 (tools/dashboard/is28_schema.js) ---------- */
  function decodeNav(p) {
    const w0 = le32(p, 0);
    return {
      state: u(w0, 0, 3), stateName: name(NAV_STATE, u(w0, 0, 3)),
      owner: u(w0, 3, 3), ownerName: name(NAV_OWNER, u(w0, 3, 3)),
      slow: u(w0, 6, 1) === 1, searchStep: u(w0, 7, 3), dropped: u(w0, 10, 8),
      commandsSent: p[4] | (p[5] << 8)
    };
  }
  function decodeSafety(p) {
    const w0 = le32(p, 0), w1 = le32(p, 4), w2 = le32(p, 8);
    const mask = u(w2, 20, 8);
    return {
      state: u(w0, 0, 2), stateName: name(SAFETY_STATE, u(w0, 0, 2)),
      inhibit: u(w0, 2, 1) === 1,
      faultSrc: name(SAFETY_SRC, u(w0, 3, 3)), warnSrc: name(SAFETY_SRC, u(w0, 6, 3)),
      faultCode: u(w0, 9, 8), warnCode: u(w0, 17, 8),
      estops: u(w1, 0, 8), faults: u(w1, 8, 8), warnings: u(w1, 16, 8),
      slowStops: u(w1, 24, 8),
      lastStopMs: u(w2, 0, 10), maxStopMs: u(w2, 10, 10),
      faultMask: mask,
      faultNames: FAULT_BITS.filter((_, i) => (mask >> i) & 1)
    };
  }
  function decodeB5(p) {
    const w0 = le32(p, 0), w1 = le32(p, 4), w2 = le32(p, 8), w3 = le32(p, 12);
    return {
      state: u(w0, 0, 4), stateName: name(B5_STATE, u(w0, 0, 4)),
      fault: u(w0, 4, 3), faultName: name(B5_FAULT, u(w0, 4, 3)),
      side: name(B5_SIDE, u(w0, 7, 2)), blockedTries: u(w0, 9, 2), recoverTries: u(w0, 11, 2),
      servoDeg: s(w0, 13, 11) / 10, found: u(w0, 24, 1) === 1,
      rangeMm: u(w1, 0, 12), closestMm: u(w1, 12, 12), confidencePct: u(w1, 24, 7),
      widthMm: u(w2, 0, 12), lateralMm: u(w2, 12, 12),
      pings: u(w3, 0, 16), noEcho: u(w3, 16, 16)
    };
  }
  function decodeLine(p) {
    const w0 = le32(p, 0), w1 = le32(p, 4), w2 = le32(p, 8);
    const ch = u(w0, 9, 6);
    const sensors = u(w0, 3, 3);
    return {
      state: u(w0, 0, 3), stateName: name(S3_STATE, u(w0, 0, 3)),
      leftBlack: (sensors & 1) === 1, rightBlack: (sensors & 2) === 2,
      barcodeBlack: (sensors & 4) === 4,
      granted: u(w0, 6, 1) === 1, side: ['none', 'left', 'right', 'both'][u(w0, 7, 2)],
      lastChar: ch === 63 ? '-' : (CODE39[ch] || '?'),
      lastResult: name(S3_RESULT, u(w0, 15, 3)), lastCmd: name(S3_CMD, u(w0, 18, 3)),
      byDistance: u(w0, 21, 1) === 1,
      barcodesOk: u(w1, 0, 8), badReads: u(w1, 8, 8), backwards: u(w1, 16, 8),
      junctions: u(w1, 24, 8), lineLost: u(w2, 0, 8), overflows: u(w2, 8, 8)
    };
  }
  /* Motion (Buddy 2): source 2, payload 20/24/32/36/37 bytes. Speeds, roll and
   * wheel-height difference are signed little-endian values; distance is
   * a signed 32-bit millimetre count. Flag bits 0/1 independently qualify
   * odometry and quasistatic side geometry. (RTOS decodeMotion.) */
  function decodeB2Motion(p) {
    const flags = p[0];
    if ((flags & 0xf0) || p[1] !== 0) return { error: 'reserved motion bits are nonzero' };
    if (p.length >= 24 && ((p[21] & 0xf0) || p[22] > p[23]))
      return { error: 'invalid ramp diagnostic state or sample counts' };
    if (p.length >= 32 && ((p.length < 37 && p[31] !== 0) || p[30] > p[23]))
      return { error: 'invalid ramp numeric diagnostics' };
    if (p.length >= 36 && ((p.length === 36 && p[35] !== 0) || p[34] > 32))
      return { error: 'invalid robust filter diagnostics' };
    const axisMadByte = (mg) => Math.min(255, Math.ceil(mg / 4));
    if (p.length === 37 &&
        Math.max(p[31], p[35], p[36]) !== axisMadByte(le32(p, 32) & 0xffff))
      return { error: 'per-axis MAD does not match maximum MAD' };
    return {
      freshOdometry: (flags & 0x01) !== 0,
      sideEstimateValid: (flags & 0x02) !== 0,
      oneSidedMode: (flags & 0x04) !== 0,
      passive: (flags & 0x08) !== 0,
      leftSpeedMmS: s(le32(p, 2), 0, 16),
      rightSpeedMmS: s(le32(p, 4), 0, 16),
      centreSpeedMmS: s(le32(p, 6), 0, 16),
      rollDeg: s(le32(p, 8), 0, 16) / 10,
      rightMinusLeftHeightMm: s(le32(p, 10), 0, 16),
      netDistanceMm: s(le32(p, 12), 0, 32),
      invalidTransitionsLeft: le32(p, 16) & 0xffff,
      invalidTransitionsRight: (le32(p, 16) >>> 16) & 0xffff,
      ...(p.length >= 24 ? {
        qualityReasons: p[20], rampLevelReady: (p[21] & 1) !== 0,
        rampCandidate: (p[21] & 2) !== 0, rampRising: (p[21] & 4) !== 0,
        rampReturning: (p[21] & 8) !== 0,
        qualityBadSamples: p[22], qualityTotalSamples: p[23]
      } : {}),
      ...(p.length >= 32 ? {
        maxSpreadMg: le32(p, 24) & 0xffff,
        maxAbsAccelMmS2: le32(p, 26) & 0xffff,
        maxAbsTurnDegS: (le32(p, 28) & 0xffff) / 10,
        rateSamples: p[30]
      } : {}),
      ...(p.length >= 36 ? {
        maxMadMg: le32(p, 32) & 0xffff, minFifoSamples: p[34]
      } : {}),
      ...(p.length === 37 ? {
        /* upper bounds within 3 mg; 1020 means >= 1020 */
        maxMadFwdMg: p[31] * 4, maxMadLatMg: p[35] * 4, maxMadUpMg: p[36] * 4
      } : {})
    };
  }
  function decodeImu(p) {
    const w0 = le32(p, 0), w1 = le32(p, 4), w2 = le32(p, 8), w3 = le32(p, 12);
    return {
      state: u(w0, 0, 3), stateName: name(IMU_STATE, u(w0, 0, 3)),
      motion: name(IMU_MOTION, u(w0, 3, 3)), event: name(IMU_EVT, u(w0, 6, 3)),
      detail: u(w0, 9, 4), valid: u(w0, 13, 1) === 1, odometry: u(w0, 14, 1) === 1,
      pitchDeg: s(w0, 15, 11) / 10, lastFault: u(w0, 26, 4),
      heightMm: u(w1, 0, 8), highestMm: u(w1, 8, 8), humps: u(w1, 16, 8),
      rejected: u(w1, 24, 8),
      shockMg: u(w2, 0, 12), shockNotifyMs: u(w2, 12, 6), shocks: u(w2, 18, 8),
      i2cErrors: u(w2, 26, 6), framesMissed: u(w3, 0, 16), eventMs: u(w3, 16, 16)
    };
  }

  /* ---------- SDK v2.3 sources 9-14 (v2.3 dashboard is28_schema.js) ---------- */
  function decodeMotion(p) {
    const flags=p[12], levels=[u16(p,32),u16(p,34),u16(p,36)], calibrationValid=!!(flags&0x02);
    return { uptimeMs: u32(p, 0), elapsedMs: u32(p, 4), runId: u16(p, 8), phase: p[10], profile: p[11],
      flags, motionActive:!!(flags&0x01), calibrationValid, automatic:!!(flags&0x04), emergencyStop:!!(flags&0x08),
      runRelativeUptime:!!(flags&0x80), recoveryState: p[13], speedL: i16(p, 14) / 10, speedR: i16(p, 16) / 10,
      targetL: i16(p, 18), targetR: i16(p, 20), pwmL: i16(p, 22), pwmR: i16(p, 24),
      adc: [u16(p, 26), u16(p, 28), u16(p, 30)], levels,
      lineMask: p[38], lineMaskAvailable:calibrationValid, distanceMm: u32(p, 39), headingCdeg: i16(p, 43) };
  }
  function decodeLineEvent(p) {
    return { uptimeMs: u32(p, 0), event: p[4], adc: [u16(p, 5), u16(p, 7), u16(p, 9)],
      levels: [u16(p, 11),u16(p, 13)], letter: ascii(p, 15), pendingLetter: ascii(p, 16),
      action: ascii(p, 17), revision: p[18], executingAction: ascii(p, 19), executingRevision: p[20] };
  }
  function decodeNavEvent(p) {
    return { uptimeMs: u32(p, 0), event: p[4], letter: ascii(p, 5), action: ascii(p, 6),
      revision: p[7], commandId: u32(p, 8), generation: u32(p, 12), fault: u16(p, 16),
      owner: p[18], mode: p[19], commandState: p[20], detail: u16(p, 21) };
  }
  function decodeSummary(p) {
    return { uptimeMs: u32(p, 0), runId: u16(p, 4), result: p[6], flags: p[7],
      elapsedMs: u32(p, 8), distanceMm: u32(p, 12), lineEvents: u16(p, 16),
      navEvents: u16(p, 18), recoveries: u16(p, 20) };
  }
  function decodeDiagnostics(p) {
    const d={uptimeMs:u32(p,0),thresholdAdc:u16(p,4),bank:p[6],flags:p[7],
      decodedTotal:u32(p,8),turns:u32(p,12),lefts:u32(p,16),rights:u32(p,20),uturns:u32(p,24),
      badFrames:u32(p,28),unsupported:u32(p,32),maxAdcGapUs:u32(p,36),maxControlGapUs:u32(p,40),
      txDropped:u32(p,44),rxDropped:u32(p,48),invalidLeft:u32(p,52),invalidRight:u32(p,56)};
    if(d.thresholdAdc>4095) return {error:'barcode threshold exceeds 4095'};
    if(d.bank>63) return {error:'barcode bank exceeds 63'};
    if(d.flags&0xf0) return {error:'diagnostics flags reserved bits are nonzero'};
    d.scanning=!!(d.flags&1); d.pending=!!(d.flags&2); d.frontCalibrationValid=!!(d.flags&4); d.runActive=!!(d.flags&8);
    return d;
  }
  /* Source 15 carries multiple layouts in SDK v2.3 integration builds; dispatch is by
   * validated payload length and subtype, never by source number alone:
   *  - ramp assist v0.1/v0.2 (+ Buddy4 estimator marker): payload 28 bytes
   *    (ramp_assist.h ra_tlm_pack; flags bits4-6 detector, bit7 estimator v0.2);
   *  - barcode peak decoder v0.1: payload 24 bytes, payload[4] = 0xB1
   *    (barcode_peak.h bp_tlm_pack; flags bit7 = combined Buddy1-4 build);
   *  - encoder fault snapshot v0.2.1: payload 45 bytes, payload[0] = 0xC4;
   *  - barcode capture metadata v0.2.1: payload 27 bytes, payload[0] = 0xC5;
   *  - network-supervision stop snapshot v0.2.1: payload 29 bytes, payload[0] = 0xC6;
   *  - junction decision v0.2.2: payload 17 bytes, payload[0] = 0xC7;
   *  - drive state v0.2.2: payload 15 bytes, payload[0] = 0xC8;
   *  - IMU lift/overturn safety snapshot v0.2.5: payload 27 bytes, payload[0] = 0xC9.
   *  - retained IMU peak evidence: payload 25 bytes, payload[0] = 0xCA.
   *  - drive-mode/load status v0.2.5: payload 14 bytes, payload[0] = 0xCB.
   * Anything else is rejected as an unknown layout. */
  const RA_EVENT = ['periodic', 'activate', 'end', 'stall push', 'floor/hold end'];   /* 3 = v0.4 stall push started, 4 = v0.5 tilt floor / v0.6 speed hold ended */
  const RA_END = ['none', 'level', 'descent', 'invalid', 'stale', 'timeout', 'not following', 'paused', 'stopped'];
  const BP_REASON = ['ok', 'too few bars', 'bar height unclear', 'wide bar count', 'unit out of range',
                     'space unclear', 'wide character gap', 'not a *X* frame', 'ambiguous', 'no wide bars',
                     'run start'];
  function i8(p, i) { return p[i] & 0x80 ? p[i] - 256 : p[i]; }
  function decodeRampAssist(p) {
    if (p[4] > 4) return { error: 'invalid ramp-assist event' };
    const fl = p[5], h = i8(p, 14), mx = i8(p, 16), mn = i8(p, 17);
    return { kind: 'ramp', uptimeMs: u32(p, 0), event: p[4], eventName: name(RA_EVENT, p[4]),
      active: !!(fl & 1), assisting: !!(fl & 2), latched: !!(fl & 4), sampleValid: !!(fl & 8),
      detector: ((fl >> 4) & 7) || 1, estimator: fl & 0x80 ? 2 : 1,
      lastEnd: p[6], lastEndName: name(RA_END, p[6]), activations: p[7], alphaMilli: u16(p, 8),
      addedPwm: [i16(p, 10), i16(p, 12)], heightMm: h === -128 ? null : h, peakMm: i8(p, 15),
      intervalMaxMm: mx === -128 ? null : mx, intervalMinMm: mn === 127 ? null : mn,
      validSamples: p[18], invalidSamples: p[19], longestOverEnter: p[20], maxAbsI: [p[21], p[22]],
      iLimitTicks: p[23], activeMs: u16(p, 24), peakAddedPwm: i16(p, 26) };
  }
  function decodeBarcodePeak(p) {
    const fl = p[5];
    return { kind: 'peak', uptimeMs: u32(p, 0), version: p[4] & 15, active: !!(fl & 1), decoded: !!(fl & 2),
      applied: !!(fl & 4), originalSameFrame: !!(fl & 8), reverse: !!(fl & 16), originalConflict: !!(fl & 32),
      overflow: !!(fl & 64), combinedBuild: !!(fl & 128), code: p[6] ? String.fromCharCode(p[6]) : '?',
      reason: p[7], reasonName: name(BP_REASON, p[7]), bars: p[8], window: p[9] & 15, removed: (p[9] >> 4) & 3,
      baseAdc: u16(p, 10), minNarrowPeak: u16(p, 12), maxWidePeak: u16(p, 14), minNarrowRise: p[16],
      maxNarrowSpaceU: p[17] / 20, minWideSpaceU: p[18] / 20, unitMm: p[19] / 50, spanMm: u16(p, 20) / 10,
      groups: u16(p, 22) };
  }
  function bitsLE(p, i, width) {
    let value = 0;
    for (let bit = 0; bit < width; bit++)
      if (p[i + (bit >>> 3)] & (1 << (bit & 7))) value += 2 ** bit;
    return value;
  }
  function signedBitsLE(p, i, width) {
    const value = bitsLE(p, i, width), sign = 2 ** (width - 1);
    return value >= sign ? value - 2 ** width : value;
  }
  function decodeEncoderFault(p) {
    if (p.length !== 45 || p[0] !== 0xC4) return { error: 'unknown encoder-fault layout' };
    if (p[1] !== 0x21) return { error: 'unsupported encoder-fault version' };
    const flags = p[2], valid = !!(flags & 1), kind = (flags >>> 1) & 3, wheel = (flags >>> 3) & 3;
    if (flags & 0xc0) return { error: 'encoder-fault reserved flag bits are nonzero' };
    if (!valid) {
      if (flags !== 0 || p.subarray(3).some((byte) => byte !== 0))
        return { error: 'invalid encoder-fault clear snapshot' };
      return { kind: 'encoder-fault', version: '0.2.1', valid: false };
    }
    if (kind < 1 || kind > 3) return { error: 'invalid encoder-fault kind' };
    if (wheel !== 1 && wheel !== 2) return { error: 'invalid encoder-fault wheel' };
    if (p[13] & 0xfe) return { error: 'encoder-fault duration reserved bits are nonzero' };
    if (p[37] & 0xfc) return { error: 'encoder-fault target reserved bits are nonzero' };
    if (p[40] & 0xc0) return { error: 'encoder-fault PWM reserved bits are nonzero' };
    const directionDurationUs = bitsLE(p, 11, 17);
    if ((kind === 3) !== (directionDurationUs !== 0))
      return { error: 'encoder-fault direction duration does not match fault kind' };
    if (kind === 3 && directionDurationUs < 50000)
      return { error: 'encoder-fault direction duration is below 50000 us' };
    /* The two 9-bit targets and two 11-bit PWM values share their packed bytes. */
    const targetPacked = bitsLE(p, 35, 18), pwmPacked = bitsLE(p, 38, 22);
    const targetL = signedBitsLE(p, 35, 9);
    const targetRightValue = Math.floor(targetPacked / 512);
    const targetR = targetRightValue >= 256 ? targetRightValue - 512 : targetRightValue;
    const pwmLeftValue = bitsLE(p, 38, 11), pwmRightValue = Math.floor(pwmPacked / 2048);
    const pwmL = pwmLeftValue >= 1024 ? pwmLeftValue - 2048 : pwmLeftValue;
    const pwmR = pwmRightValue >= 1024 ? pwmRightValue - 2048 : pwmRightValue;
    if (targetL < -180 || targetL > 180 || targetR < -180 || targetR > 180)
      return { error: 'encoder-fault target speed is out of range' };
    if (pwmL < -1000 || pwmL > 1000 || pwmR < -1000 || pwmR > 1000)
      return { error: 'encoder-fault PWM is out of range' };
    const badChanges = bitsLE(p, 14, 40);
    return { kind: 'encoder-fault', version: '0.2.1', valid: true, saturated: !!(flags & 0x20),
      fault: ['none', 'error burst', 'error density', 'wrong direction'][kind],
      wheel: wheel === 1 ? 'left' : 'right', controlTimerUs: u32(p, 3), kernelMs: u32(p, 7),
      directionDurationUs, windowBad: badChanges & 0xfffff, windowChanges: Math.floor(badChanges / 0x100000),
      leftEdges: u32(p, 19), leftInvalid: u32(p, 23), rightEdges: u32(p, 27), rightInvalid: u32(p, 31),
      targetLeft: targetL, targetRight: targetR, pwmLeft: pwmL, pwmRight: pwmR,
      speedLeft: i16(p, 41) / 10, speedRight: i16(p, 43) / 10 };
  }
  function decodeBarcodeCapture(p) {
    if (p.length !== 27 || p[0] !== 0xC5) return { error: 'unknown barcode-capture layout' };
    const meta = u32(p, 23), flags = meta & 63, codeIndex = (meta >>> 6) & 31;
    const reason = (meta >>> 11) & 15, bars = (meta >>> 15) & 31, rawCount = (meta >>> 20) & 0xfff;
    if (codeIndex > 26) return { error: 'barcode-capture code index exceeds 26' };
    if (reason > 10) return { error: 'barcode-capture result exceeds 10' };
    if (bars > 24) return { error: 'barcode-capture bar count exceeds 24' };
    if (rawCount > 2048) return { error: 'barcode-capture raw count exceeds 2048' };
    if (reason === 10 && (meta & 4)) return { error: 'in-progress barcode capture cannot be finalized' };
    if (!!(flags & 2) !== (codeIndex !== 0) || ((flags & 32) && !(flags & 2)))
      return { error: 'barcode-capture known code does not match decoder flags' };
    if (!(flags & 1) && (meta !== 0 || p.subarray(5,21).some(byte => byte !== 0)))
      return { error: 'invalid barcode-capture clear snapshot' };
    return { kind: 'barcode-capture', uptimeMs: u32(p, 1), attemptSequence: u32(p, 5),
      firstRawMs: u32(p, 9), lastRawMs: u32(p, 13), rawOverwritten: u32(p, 17), runId: u16(p, 21),
      valid: !!(flags & 1), decoded: !!(flags & 2), finalized: !!(flags & 4), truncated: !!(flags & 8),
      startValid: !!(flags & 16), originalAccepted: !!(flags & 32),
      code: codeIndex ? String.fromCharCode(64 + codeIndex) : '?', reason,
      reasonName: reason === 10 ? 'in progress' : BP_REASON[reason], bars, rawCount };
  }
  function decodeNetworkStop(p) {
    if (p.length !== 29 || p[0] !== 0xc6) return { error: 'unknown network-stop layout' };
    if (p[1] !== 0x21) return { error: 'unsupported network-stop version' };
    if (p[2] !== 0) return { error: 'network-stop reserved byte is nonzero' };
    const meta = u16(p, 27);
    if (meta & 0xc000) return { error: 'network-stop reserved metadata bits are nonzero' };
    const valid = !!(meta & 0x2000), flags = meta & 7;
    if (!valid) {
      if (p.subarray(2).some((byte) => byte !== 0)) return { error: 'invalid network-stop clear snapshot' };
      return { kind: 'network-stop', version: '0.2.1', valid: false };
    }
    let link = (meta >>> 3) & 7;
    if (link & 4) link -= 8;
    if (link < -3) return { error: 'network-stop link code is outside -3..3' };
    if (((meta >>> 6) & 127) === 0) return { error: 'network-stop valid snapshot has no failure condition' };
    return { kind: 'network-stop', version: '0.2.1', valid: true,
      stopMs: u32(p, 3), pollAgeMs: u32(p, 7), appAgeMs: u32(p, 11), ackAgeMs: u32(p, 15),
      connects: u32(p, 19), lastCompleteMs: u32(p, 23),
      configured: !!(flags & 1), connected: !!(flags & 2), subscribed: !!(flags & 4),
      link, failMask: (meta >>> 6) & 0x7f };
  }
  function decodeJunctionDecision(p) {
    if (p.length!==17 || p[0]!==0xc7 || (p[13]&0xf0)) return {error:'invalid junction-decision layout'};
    const threshold=u16(p,11)&4095, hysteresis=(p[12]>>>4)|((p[13]&15)<<4), flags=p[16], result=flags&7;
    if(result>5 || hysteresis>205) return {error:'invalid junction-decision fields'};
    return {kind:'junction-decision',uptimeMs:u32(p,1),decisionSerial:u32(p,5),runId:u16(p,9),
      threshold,hysteresis,blackEntry:threshold+hysteresis,whiteReturn:threshold-hysteresis,
      widthMm:i16(p,14)/100,result,resultName:['no stripe measured','awaiting trailing white','too narrow','too wide','white confirmed','white interrupted'][result],
      valid:!!(flags&8),armed:!!(flags&16),pending:!!(flags&32),candidate:!!(flags&64),saturated:!!(flags&128)};
  }
  function decodeDriveState(p) {
    if(p.length!==15 || p[0]!==0xc8 || p[1]!==0x22) return {error:'invalid drive-state layout'};
    const meta=u16(p,8), angular=u16(p,10), targets=p[12]|(p[13]<<8)|(p[14]<<16);
    const side=(meta>>>2)&3,percent=(meta>>>4)&127,stage=angular&7;
    const left=s(targets,0,9),right=s(targets,9,9);
    if((meta&0xc000)||(targets>>>19)||side>2||percent>100||stage>4||Math.abs(left)>180||Math.abs(right)>180)
      return {error:'invalid drive-state fields'};
    return {kind:'drive-state',version:'0.2.2',uptimeMs:u32(p,2),runId:u16(p,6),bendPhase:meta&3,
      bendSide:side,bendPercent:percent,recoveryPhase:(meta>>>11)&7,sweepStage:stage,
      recoveryYawDeg:s(angular,3,13)/10,targetLeft:left,targetRight:right,saturated:!!(targets&(1<<18))};
  }
  /* C9 codec 0x23 is retained for old firmware; codec 0x25 extends the
   * accepted tilt dwell to 100 ms while preserving the rest of the layout. */
  const IMU_STOP_REASON = ['none', 'lifted', 'overturn', 'unavailable'];
  function decodeImuLiftSafety(p) {
    if (p.length !== 27 || p[0] !== 0xc9) return { error: 'unknown IMU lift-safety layout' };
    if (p[1] !== 0x23 && p[1] !== 0x25) return { error: 'unsupported IMU lift-safety version' };
    if ((p[17] & 0xe0) || (p[19] & 0xf0) || (p[20] & 0x80) ||
        (p[21] & 0xe0) || (p[22] & (p[1] === 0x25 ? 0x80 : 0xc0)) || (p[24] & 0xfe))
      return { error: 'IMU lift-safety reserved bits are nonzero' };
    const reason = p[20] & 3;
    const upDwellMs = p[21], tiltDwellMs = p[22] & 0x7f, uprightMs = u16(p, 23);
    if (reason > 3 || upDwellMs > 30 || tiltDwellMs > (p[1] === 0x25 ? 100 : 50) || uprightMs > 500)
      return { error: 'IMU lift-safety value is out of range' };
    let upProjection = u16(p, 16) & 0x1fff;
    if (upProjection & 0x1000) upProjection -= 0x2000;
    const flags = p[20];
    return { kind: 'imu-lift-safety', version: p[1] === 0x25 ? '0.2.5' : '0.2.3', codec: p[1], uptimeMs: u32(p, 2),
      tripMs: u32(p, 6), lastSampleMs: u32(p, 10), eventCount: u16(p, 14),
      upProjection, normMg: u16(p, 18) & 0x0fff,
      reason, reasonName: IMU_STOP_REASON[reason], latched: !!(flags & 0x04),
      armed: !!(flags & 0x08), fresh: !!(flags & 0x10), canReset: !!(flags & 0x20),
      saturated: !!(flags & 0x40), upDwellMs, tiltDwellMs, uprightMs, runId: u16(p, 25) };
  }
  function decodeImuPeakEvidence(p) {
    if (p.length !== 25 || p[0] !== 0xca) return { error: 'unknown IMU peak-evidence layout' };
    if (p[1] !== 1) return { error: 'unsupported IMU peak-evidence codec' };
    const normPacked = u16(p, 12), peakPacked = u16(p, 14), flags = p[18];
    if ((normPacked & 0xf000) || (peakPacked & 0xe000) || (p[16] & 0x80) ||
        (p[17] & 0xe0) || ((flags & 3) > 3) || (((flags >>> 2) & 3) > 3))
      return { error: 'invalid IMU peak-evidence fields' };
    let peakUp = peakPacked & 0x1fff;
    if (peakUp & 0x1000) peakUp -= 0x2000;
    const candidate = flags & 3, rejectReason = (flags >>> 2) & 3;
    const peakSampleMs = u32(p, 6), windowStartMs = u32(p, 19), eventCount = u16(p, 23);
    const minCosQ15 = i16(p, 10), normMg = normPacked & 0x0fff;
    return { kind: 'imu-lift-peak', codec: p[1], reportMs: u32(p, 2), peakSampleMs,
      hasPeak: !(peakSampleMs === 0 && windowStartMs === 0 && eventCount === 0),
      minCosQ15, minCos: minCosQ15 / 32768,
      peakAngleDeg: Math.acos(Math.max(-1, Math.min(1, minCosQ15 / 32768))) * 180 / Math.PI,
      normMg, peakUpMg: peakUp, maxTiltDwellMs: p[16], maxUpDwellMs: p[17] & 0x1f,
      candidate, rejectReason, lastCandidate: candidate, lastRejection: rejectReason,
      armed: !!(flags & 0x10), currentValid: !!(flags & 0x20),
      latched: !!(flags & 0x40), saturated: !!(flags & 0x80),
      windowStartMs, eventCount };
  }

  /* ---------- Buddy5 RTOS integration (rtos_buddy1_5 v0.1) ----------
   * All are source-15 layouts dispatched by EXACT payload length AND marker
   * byte 4 (none has length 28, so the ramp-assist rule is unaffected):
    *  E5 mission status v1 (54) / v2 (59)   D1 width record v4 (56)  D2 width summary v4 (58)
   *  D3 width result v1 (58)     D4 rejected triplet v1 (32)
   *  BE direction record v2/v3 (48)  BF direction summary v2/v3 (44)
   *  B5 bypass/rejoin event v1 (32)
   * Width is a DISPLAY estimate only; it never authorises motion. */
  const B5M_STAGE = ['idle', 'guard', 'halting', 'width probe', 'line alignment', 'approach',
                     'direction probe', 'bypass', 'hold', 'cancelled', 'fresh range'];
  const B5M_REASON = ['none', 'cancelled', 'sonar fault', 'lease lost', 'move refused/interrupted',
    'stage timeout', 'stale observations/coordinator fault', 'no encoder progress', 'width probe aborted',
    'width probe could not start', 'no line evidence after alignment search', 'line lost during approach',
    'approach range not reached', 'direction probe aborted', 'direction unknown/contradictory',
    'direction probe could not start', 'fresh range unusable', 'bypass blocked (range/clearance)',
    'bypass/rejoin fault'];
  const B5M_BASIS = ['none', 'probe evidence', 'DEFAULT side (healthy no preference)', 'held (no movement)'];
  const B5M_SIDE = ['none', 'right', 'left'];
  /* E5 v2 byte 19. "skipped" is a DELIBERATE no-attempt on a clean stopped
   * range outside 270-330 mm; "failed" is an attempted probe that held. */
  const B5M_WIDTH_POLICY = ['none', 'measured', 'skipped (clean range outside 270-330 mm, no probe)',
                            'failed (attempted, HOLD)', 'probe running'];
  const B5_MODE = ['AUTO', 'STOPPING', 'PAUSED', 'DRIVING', 'TURNING', 'SETTLING', 'FAULT', 'OFF'];
  const B5_OWNER = ['none', 'Buddy1', 'Buddy2', 'Buddy3 (line)', 'Buddy4', 'Buddy5 (obstacle)', '?', '?'];
  const B5WP_END = ['NONE', 'COMPLETE', 'CANCELLED', 'MOVE', 'SENSOR', 'LEASE', 'TIMEOUT', 'REFERENCE',
                    'ALIGNMENT', 'TRAVEL', 'CLEARANCE'];
  const B5EP_END = ['NONE', 'COMPLETE', 'CANCELLED', 'MOVE', 'SENSOR', 'LEASE', 'TIMEOUT', 'REFERENCE', 'ALIGNMENT'];
  const B5WE_BASIS = ['NONE', 'CALIBRATED_RANGE_RISE', 'EMPIRICAL_REACH_COMPLETION', 'PROJECT_RANGE_ASSUMPTION'];
  const B5WE_STATUS = ['NONE', 'ESTIMATE', 'REACH_LIMIT', 'INVALID'];
  const B5WE_REASON = ['NONE', 'NO_RECORDS', 'INCONSISTENT', 'RUN_OR_REFERENCE', 'EDGES_UNRESOLVED'];
  const B5WE_EDGE = ['NO_DATA', 'ONSET', 'HIGH_FROM_START', 'WALK_FREE'];
  const B5EP_SIDE = ['UNKNOWN', 'OPEN', 'CROSSED', 'REMAINS'];
  const B5EP_DECISION = ['HOLD', 'LEFT', 'RIGHT'];
  const B5A_STATE = ['IDLE', 'GUARD', 'CONFIRM', 'HALTING', 'SCAN', 'REFINE', 'PLAN', 'BYPASS', 'REJOIN',
                     'BLOCKED', 'FAULT', 'GEOMETRY_HOLD', 'DONE'];
  const B5A_REASON = ['OK', 'SENSOR', 'LEASE', 'CANCELLED', 'NO_PATH', 'MOVE', 'NO_LINE', 'STALE',
    'NO_ENCODER_PROGRESS', 'GEOMETRY_DONE', 'INSUFFICIENT_TURN_CLEARANCE', 'NO_LINE_AFTER_FALLBACK_SWEEP'];
  const B5R_PHASE = ['START', 'RETURN', 'RETURN_STOP', 'ADVANCE', 'RESTORE', 'RESTORE_TURN', 'CONFIRM',
                     'SEARCH', 'SEARCH_STOP', 'RESUME', 'ACQUIRE', 'ACQUIRE_STOP'];
  const B5V_EVENT = ['NONE', 'TAKEOVER', 'PLAN', 'HOLD_CLEARANCE', 'LEG_END', 'RETURN_START', 'CANDIDATE',
    'REJECT', 'STRIPE_FOUND', 'POSITIONED', 'RESTORED', 'SEARCH_TURN', 'CONFIRMED', 'HANDBACK', 'STOPPED',
    'GEOMETRY_HOLD', 'ACQUIRE', 'ACQUIRED', 'RETURN_FALLBACK'];
  const B5_NONE16 = 0xffff;
  function b5mm100(v) { return v === B5_NONE16 ? null : v / 100; }
  function b5mm10(v) { return v === B5_NONE16 ? null : v / 10; }
  function b5pings(p, cls, val) {
    const out = [];
    for (let k = 0; k < 3; k++) {
      const c = p[cls + k] & 3, why = (p[cls + k] >> 4) & 7, v = u16(p, val + 2 * k);
      out.push(c === 0 ? { cls: 'echo', mm: v } : c === 1 ? { cls: 'no-return', widthUs: v }
        : { cls: 'fault', why, widthUs: v });
    }
    return out;
  }
  function decodeB5MissionStatus(p) {
    if (p[4] !== 0xE5 || !((p.length === 54 && p[5] === 1) || (p.length === 59 && p[5] === 2))) {
      if ((p.length === 54 || p.length === 59) && p[4] === 0xE5)
        return { error: 'unsupported Buddy5 status version ' + p[5] + ' for length ' + p.length };
      return { error: 'unknown Buddy5 status layout' };
    }
    const v2 = p[5] === 2;
    const flags = u16(p, 12);
    if ((flags & 0xf000) || (!v2 && p[19] !== 0)) return { error: 'Buddy5 status reserved bits are nonzero' };
    if (v2 && (p[19] >= B5M_WIDTH_POLICY.length || Math.abs(i8(p, 54)) > 80 || p[55] > 80))
      return { error: 'Buddy5 status v2 field out of range' };
    if (p[8] >= B5M_STAGE.length || p[9] >= B5M_REASON.length || p[25] > 3 || p[28] > 2 || p[24] > 2)
      return { error: 'Buddy5 status field out of range' };
    const co = p[11], pingMm = u16(p, 14), pingAge = u16(p, 16);
    const policy = v2 ? p[19] : null;
    return { kind: 'b5-status', version: p[5], uptimeMs: u32(p, 0), run: u16(p, 6),
      widthPolicy: policy, widthPolicyName: v2 ? B5M_WIDTH_POLICY[policy] : 'n/a (v1)',
      widthSkipped: policy === 2, widthFailed: policy === 3,
      steerDeg: v2 ? i8(p, 54) / 10 : null, steerMaxDeg: v2 ? p[55] / 10 : null,
      steerSaturated: v2 ? p[56] : null, widthGateMm: v2 ? (u16(p, 57) || null) : null,
      stage: p[8], stageName: B5M_STAGE[p[8]], reason: p[9], reasonName: B5M_REASON[p[9]], moduleReason: p[10],
      owner: co & 7, ownerName: B5_OWNER[co & 7], mode: (co >> 3) & 7, modeName: B5_MODE[(co >> 3) & 7],
      sessionActive: !!(co & 0x40), stopped: !!(co & 0x80),
      armed: !!(flags & 1), rearm: !!(flags & 2), pingFresh: !!(flags & 4), widthAttempted: !!(flags & 8),
      widthResult: !!(flags & 16), directionAttempted: !!(flags & 32), defaultSide: !!(flags & 64),
      bypassActive: !!(flags & 128), backlog: !!(flags & 256), servoSettled: !!(flags & 512),
      hold: !!(flags & 1024), ready: !!(flags & 2048), flags,
      pingMm: pingMm === B5_NONE16 ? null : pingMm, pingAgeMs: pingAge === B5_NONE16 ? null : pingAge,
      pingClass: ['echo', 'no-return', 'fault', '?'][p[18] & 3], pingWhy: (p[18] >> 2) & 15,
      servoRequestedDeg: i16(p, 20) / 10, servoAppliedDeg: i16(p, 22) / 10,
      decision: p[24], decisionName: B5EP_DECISION[p[24]], basis: p[25], basisName: B5M_BASIS[p[25]],
      stateL: p[26] & 15, stateR: p[26] >> 4, stateLName: B5EP_SIDE[p[26] & 15] || '?',
      stateRName: B5EP_SIDE[p[26] >> 4] || '?', directionEnd: p[27], directionEndName: B5EP_END[p[27]] || '?',
      side: p[28], sideName: B5M_SIDE[p[28]], widthEnd: p[29], widthEndName: B5WP_END[p[29]] || '?',
      widthBasis: p[30], widthBasisName: B5WE_BASIS[p[30]] || '?', widthStatus: p[31],
      widthStatusName: B5WE_STATUS[p[31]] || '?', widthReason: p[32], widthReasonName: B5WE_REASON[p[32]] || '?',
      widthRetries: p[33], widthDisplayMm: b5mm100(u16(p, 34)), widthRawMm: b5mm100(u16(p, 36)),
      widthCorrectedMm: b5mm100(u16(p, 38)), widthD0Mm: u16(p, 40), triggerMm: u16(p, 42),
      bypassRangeMm: u16(p, 44), bypassState: p[46], bypassStateName: B5A_STATE[p[46]] || '?',
      rejoinPhase: p[47], rejoinPhaseName: B5R_PHASE[p[47]] || '?',
      widthProduced: p[48], widthAdmitted: p[49], directionProduced: p[50], directionAdmitted: p[51],
      approachLegs: p[52] & 15, alignSteps: p[52] >> 4, discarded: p[53] };
  }
  function decodeB5WidthRecord(p) {
    if (p[5] !== 4) return { error: 'unsupported D1 width record version ' + p[5] };
    return { kind: 'b5-width-record', version: p[5], uptimeMs: u32(p, 0), run: u16(p, 6), index: u16(p, 8),
      phase: p[10], side: p[11], step: p[12], n: p[13], headingDeg: i16(p, 14) / 100,
      servoDeg: i16(p, 16) / 10, appliedDeg: i16(p, 18) / 10, pathMm: i16(p, 20) / 10,
      expectedMm: u16(p, 26), anchorMm: u16(p, 28), pings: b5pings(p, 32, 36), evidence: p[35],
      d0: u16(p, 42), lost: u16(p, 44), netXMm: i16(p, 46) / 10, netYMm: i16(p, 48) / 10,
      axisXMm: i16(p, 52) / 10, frontGapMm: i16(p, 54) / 10 };
  }
  function decodeB5WidthSummary(p) {
    if (p[5] !== 4) return { error: 'unsupported D2 width summary version ' + p[5] };
    return { kind: 'b5-width-summary', version: p[5], uptimeMs: u32(p, 0), run: u16(p, 6), end: p[8],
      endName: B5WP_END[p[8]] || '?', widthResult: p[9], stateL: p[10], stateR: p[11], stepsL: p[12],
      stepsR: p[13], records: u16(p, 14), lost: u16(p, 16), d0: u16(p, 18), flags: u16(p, 38),
      finalHeadingDeg: i16(p, 32) / 100, netXMm: i16(p, 34) / 10, netYMm: i16(p, 36) / 10,
      corrections: u16(p, 40) };
  }
  function decodeB5WidthResult(p) {
    if (p[5] !== 1) return { error: 'unsupported D3 width result version ' + p[5] };
    if (p[9] > 3 || p[10] > 3 || p[11] > 4) return { error: 'D3 width result field out of range' };
    const side = (o) => ({ status: p[o], statusName: B5WE_EDGE[p[o]] || '?', n: p[o + 1],
      aMm: b5mm10(u16(p, o + 2)), bMm: b5mm10(u16(p, o + 4)), baseMm: i16(p, o + 6) / 4 });
    return { kind: 'b5-width-result', version: p[5], uptimeMs: u32(p, 0), run: u16(p, 6), end: p[8],
      endName: B5WP_END[p[8]] || '?', basis: p[9], basisName: B5WE_BASIS[p[9]], status: p[10],
      statusName: B5WE_STATUS[p[10]], reason: p[11], reasonName: B5WE_REASON[p[11]],
      displayMm: b5mm100(u16(p, 12)), rawMm: b5mm100(u16(p, 14)), correctedMm: b5mm100(u16(p, 16)),
      sampledMm: b5mm10(u16(p, 18)), spanLoMm: b5mm10(u16(p, 20)), spanHiMm: b5mm10(u16(p, 22)),
      flags: u16(p, 24), left: side(26), right: side(34), d0: u16(p, 42), records: u16(p, 44),
      lost: u16(p, 46), offsetMm: u16(p, 48) / 10, rangeLoMm: u16(p, 50), rangeHiMm: u16(p, 52),
      ruleId: p[56], widthResult: p[57], measured: p[9] === 1 || p[9] === 2, assumed: p[9] === 3 };
  }
  function decodeB5WidthRetry(p) {
    if (p[5] !== 1) return { error: 'unsupported D4 retry version ' + p[5] };
    return { kind: 'b5-width-retry', version: p[5], uptimeMs: u32(p, 0), run: u16(p, 6), phase: p[8],
      side: p[9], step: p[10], attempt: p[11], expectedMm: u16(p, 12), headingDeg: i16(p, 14) / 100,
      appliedDeg: i16(p, 16) / 10, pings: b5pings(p, 18, 22), dwellMs: u16(p, 28) };
  }
  function decodeB5DirectionRecord(p) {
    if (p[5] !== 2 && p[5] !== 3) return { error: 'unsupported BE direction record version ' + p[5] };
    return { kind: 'b5-direction-record', version: p[5], nearMm: p[5] === 3 ? 25 : 15, uptimeMs: u32(p, 0),
      run: u16(p, 6), index: u16(p, 8), phase: p[10], n: p[11], headingDeg: i16(p, 12) / 100,
      servoDeg: i16(p, 14) / 10, appliedDeg: i16(p, 16) / 10, travelMm: i16(p, 18) / 10,
      expectedMm: u16(p, 20), anchorMm: u16(p, 22), pings: b5pings(p, 24, 28), evidence: p[27],
      d0: u16(p, 34), lost: u16(p, 36), netXMm: i16(p, 38) / 10, netYMm: i16(p, 40) / 10 };
  }
  function decodeB5DirectionSummary(p) {
    if (p[5] !== 2 && p[5] !== 3) return { error: 'unsupported BF direction summary version ' + p[5] };
    if (p[9] > 2 || p[10] > 3 || p[11] > 3) return { error: 'BF direction field out of range' };
    return { kind: 'b5-direction-summary', version: p[5], nearMm: p[5] === 3 ? 25 : 15, uptimeMs: u32(p, 0),
      run: u16(p, 6), end: p[8], endName: B5EP_END[p[8]] || '?', decision: p[9], decisionName: B5EP_DECISION[p[9]],
      stateL: p[10], stateLName: B5EP_SIDE[p[10]], stateR: p[11], stateRName: B5EP_SIDE[p[11]],
      records: u16(p, 12), lost: u16(p, 14), d0: u16(p, 16), finalHeadingDeg: i16(p, 32) / 100,
      netXMm: i16(p, 34) / 10, netYMm: i16(p, 36) / 10, flags: u16(p, 38), corrections: u16(p, 40) };
  }
  function decodeB5Event(p) {
    if ((p[5] & 15) !== 1) return { error: 'unsupported B5 event version ' + (p[5] & 15) };
    return { kind: 'b5-event', version: 1, uptimeMs: u32(p, 0), geometryOnly: !!(p[5] & 16), code: p[6],
      codeName: B5V_EVENT[p[6]] || '?', state: p[7], stateName: B5A_STATE[p[7]] || '?', phase: p[8],
      phaseName: B5R_PHASE[p[8]] || '?', reason: p[9], reasonName: B5A_REASON[p[9]] || '?', result: p[10],
      detail: p[11], searches: p[12], rejects: p[13], a: i16(p, 14) / 10, b: i16(p, 16) / 10,
      headingDeg: i16(p, 18) / 100, level: [u16(p, 20), u16(p, 22)], bg: [u16(p, 24), u16(p, 26)],
      eventSeq: u16(p, 28), overwritten: u16(p, 30) };
  }
  function decodeB5Source15(p) {
    if (p.length < 6) return null;
    if ((p.length === 54 || p.length === 59) && p[4] === 0xE5) return decodeB5MissionStatus(p);
    if (p.length === 56 && p[4] === 0xD1) return decodeB5WidthRecord(p);
    if (p.length === 58 && p[4] === 0xD2) return decodeB5WidthSummary(p);
    if (p.length === 58 && p[4] === 0xD3) return decodeB5WidthResult(p);
    if (p.length === 32 && p[4] === 0xD4) return decodeB5WidthRetry(p);
    if (p.length === 48 && p[4] === 0xBE) return decodeB5DirectionRecord(p);
    if (p.length === 44 && p[4] === 0xBF) return decodeB5DirectionSummary(p);
    if (p.length === 32 && p[4] === 0xB5) return decodeB5Event(p);
    return null;
  }
  /* Per-run record completeness from what the dashboard actually RECEIVED.
   * Never fills a gap: missing indices are reported, and an incomplete run is
   * a telemetry problem, not a reason to move. */
  function B5Tracker() {
    const width = new Map(), direction = new Map();
    function slot(map, run) {
      if (!map.has(run)) map.set(run, { run, indices: new Map(), duplicates: 0, summary: null, result: null,
                                         retries: [] });
      return map.get(run);
    }
    function add(d) {
      if (!d || !d.kind) return;
      if (d.kind === 'b5-width-record' || d.kind === 'b5-direction-record') {
        const r = slot(d.kind === 'b5-width-record' ? width : direction, d.run);
        if (r.indices.has(d.index)) r.duplicates++; else r.indices.set(d.index, d);
      } else if (d.kind === 'b5-width-summary') slot(width, d.run).summary = d;
      else if (d.kind === 'b5-width-result') slot(width, d.run).result = d;
      else if (d.kind === 'b5-width-retry') slot(width, d.run).retries.push(d);
      else if (d.kind === 'b5-direction-summary') slot(direction, d.run).summary = d;
    }
    function report(map, run) {
      const r = map.get(run);
      if (!r) return null;
      const claimed = r.summary ? r.summary.records : (r.result ? r.result.records : null);
      const missing = [];
      if (claimed !== null) for (let k = 0; k < claimed; k++) if (!r.indices.has(k)) missing.push(k);
      return { run, received: r.indices.size, claimed, missing, duplicates: r.duplicates,
               summary: !!r.summary, result: !!r.result, retries: r.retries.length,
               complete: claimed !== null && missing.length === 0 && !!r.summary };
    }
    function latest(map) { let k = null; for (const run of map.keys()) k = run; return k; }
    return { add, width: (run) => report(width, run === undefined ? latest(width) : run),
             direction: (run) => report(direction, run === undefined ? latest(direction) : run),
             widthRun: (run) => width.get(run === undefined ? latest(width) : run) || null,
             directionRun: (run) => direction.get(run === undefined ? latest(direction) : run) || null };
  }
  function decodeSource15(p) {
    if ((p.length === 12 || p.length === 13) && p[0] === 0xC2) {
      if ((p.length === 13 && p[12] > 15) || (p[2] & 0x80) || (p[3] & 0xe0) || !(p[3] & 3)) return {error:'invalid release metadata'};
      const levels = [];
      for (let k=0;k<4;k++) {
        let n=0; for(let b=0;b<12;b++) n |= ((p[4+Math.floor((12*k+b)/8)] >>> ((12*k+b)%8))&1)<<b;
        levels.push(n);
      }
      return {kind:'release',version:`${p[1]&15}.${p[1]>>>4}.${p[2]&15}`,
        wireless:!!(p[2]&16),storedLogs:!!(p[2]&32),calibrationValid:!!(p[2]&64),
        profile:p[3]&3,calibrationOrigin:(p[3]>>>2)&3,recordedCalibration:!!(p[3]&16), motorStopReason:p.length === 13 ? p[12] : null,
        white:levels.slice(0,2),black:levels.slice(2),runId:u16(p,10)};
    }
    if (p.length === 60 && p[0] === 0xC3) {
      if ((p[58]&0xc0) || (p[57]&15)>10 || (p[57]>>>4)>10) return {error:'invalid RTOS runtime flags'};
      const keys=['uptimeMs','reads','failed','lateGaps','fifoOverruns','fifoBudget','initBudget',
        'controlCount','controlLate','holds','recoveries','disarms','earlyClears','epoch'];
      const d={kind:'runtime'}; keys.forEach((k,i)=>d[k]=u32(p,1+4*i));
      return Object.assign(d,{clearReason:p[57]&15,failureReason:p[57]>>>4,
        holdReason:p[58]&3,qualificationFlags:(p[58]>>>2)&15,controlFlags:p[59],
        imuStop:!!(p[59]&0x80)});
    }
    if (p.length === 21 && p[4] === 0xC1) {
      if ((p[5] & ~15) || p[6] > 200 || u16(p,7) > 6000 || u16(p,9) > 1000 || p[19] > 5 || p[20] > 200)
        return {error: 'invalid coordinator ramp-target fields'};
      return {kind:'ramp-targets',uptimeMs:u32(p,0),provisional:!!(p[5]&8),holdActive:!!(p[5]&1),
        boostMilli:p[6],pushPwm:u16(p,7)/100,holdMilli:u16(p,9),
        requested:[i16(p,11)/10,i16(p,13)/10],effective:[i16(p,15)/10,i16(p,17)/10],owner:p[19],ttlMs:p[20]};
    }
    if (p.length === 45 && p[0] === 0xC4) return decodeEncoderFault(p);
    if (p.length === 27 && p[0] === 0xC5) return decodeBarcodeCapture(p);
    if (p.length === 29 && p[0] === 0xc6) return decodeNetworkStop(p);
    if (p.length === 17 && p[0] === 0xc7) return decodeJunctionDecision(p);
    if (p.length === 15 && p[0] === 0xc8) return decodeDriveState(p);
    if (p.length === 27 && p[0] === 0xc9) return decodeImuLiftSafety(p);
    if (p.length === 25 && p[0] === 0xca) return decodeImuPeakEvidence(p);
    if (p.length === 37 && p[0] === 0xcc) return decodeCalibrationStatus(p);
    if (p.length === 14 && p[0] === 0xcb) return decodeDriveModeStatus(p);
    if (p.length === 28) return decodeRampAssist(p);
    if (p.length === 24 && p[4] === 0xB1) return decodeBarcodePeak(p);
    const b5 = decodeB5Source15(p);
    if (b5) return b5;
    return { error: 'unknown source-15 layout (payload ' + p.length + ' bytes' + (p.length > 4 ? ', byte4 0x' + p[4].toString(16) : '') + ')' };
  }
  function decodeCalibrationStatus(p) {
    if (p.length !== 37 || p[0] !== 0xcc || p[1] !== 0x28) return {error:'unsupported calibration layout'};
    const operation=p[9]&7, result=p[9]>>>4;
    if ((p[9]&8) || result>12 || (p[10]&0xc0) || (p[11]&0xe0) || p[12]>32 || p[13]>3 || p[36]>5)
      return {error:'invalid calibration fields'};
    const bits=(at,width)=>{let v=0;for(let i=0;i<width;i++)v|=((p[14+((at+i)>>>3)]>>>((at+i)&7))&1)<<i;return v;};
    const values=Array.from({length:8},(_,i)=>bits(i*12,12));
    const raw=u32(Uint8Array.of(p[26],p[27],p[28],0),0), spread=u32(Uint8Array.of(p[33],p[34],p[35],0),0);
    return {kind:'front-calibration',codec:0x28,uptimeMs:u32(p,2),revision:u16(p,6),requestId:p[8],operation,result,
      session:!!(p[10]&1),valid:!!(p[10]&2),adcReady:!!(p[10]&4),canChange:!!(p[10]&8),
      applied:!!(p[10]&16),saved:!!(p[10]&32),captured:p[11]&7,capturing:(p[11]>>>3)&3,samples:p[12],origin:p[13],
      activeWhite:values.slice(0,2),activeBlack:values.slice(2,4),stagedWhite:values.slice(4,6),stagedBlack:values.slice(6,8),
      raw:[raw&4095,raw>>>12],sampleMs:u32(p,29),spread:[spread&4095,spread>>>12],ack:p[36]};
  }
  function calibrationArg(operation) {
    if (!Number.isInteger(operation) || operation<0 || operation>7) throw new RangeError('calibration operation must be 0..7');
    return CFG_CALIBRATION | (operation<<8);
  }
  function decodeDriveModeStatus(p) {
    if (p.length !== 14 || p[0] !== 0xcb) return { error: 'unknown drive-mode status layout' };
    if (p[1] !== 0x25) return { error: 'unsupported drive-mode status codec' };
    if ((p[10] & 3) > 2 || ((p[10] >>> 2) & 3) > 3 || ((p[10] >>> 4) & 7) > 5 ||
        (p[11] & 0xf8) || (p[13] & 0xc0)) return { error: 'invalid drive-mode status fields' };
    const power = u16(p, 12);
    return { kind: 'drive-mode-status', version: '0.2.5', uptimeMs: u32(p, 2), revision: u32(p, 6),
      mode: p[10] & 3, requestedMode: (p[10] >>> 2) & 3, lastResult: (p[10] >>> 4) & 7,
      canChange: !!(p[10] & 0x80), sessionActive: !!(p[11] & 1),
      loadLeft: !!(p[11] & 2), loadRight: !!(p[11] & 4),
      addedPwmLeft: power & 0x7f, addedPwmRight: (power >>> 7) & 0x7f };
  }
  function decodeStoredLine(r) {
    if(r.length!==56) return {error:'RTOS controller row must be 56 bytes'};
    const a=u32(r,12),b=u32(r,16),m=u32(r,36),x=u32(r,44),y=u32(r,48);
    const angle=(y>>>18)&8191,route=[0,65,70,90];
    return {sequence:u32(r,40),ms:u32(r,0),ticks_l:(u32(r,4)|0),ticks_r:(u32(r,8)|0),
      speed_l_x10:i16(r,20),speed_r_x10:i16(r,22),target_l:i16(r,24),target_r:i16(r,26),
      pwm_l:i16(r,28),pwm_r:i16(r,30),adc_l:a&4095,adc_r:(a>>>12)&4095,
      adc_side:(a>>>24)|((b&15)<<8),level_l:(b>>>4)&1023,level_r:(b>>>14)&1023,
      phase:(b>>>24)&15,decision:m&63,reason:(m>>>6)&63,side_black:(m>>>12)&1,
      busy:(m>>>13)&1,white:(m>>>14)&1,search:(m>>>15)&1,pending:route[(m>>>16)&3],
      action:route[(m>>>18)&3],dt_us:u16(r,32),cost_us:u16(r,34),filtered_l:x&4095,
      filtered_r:(x>>>12)&4095,optical_target_l:i16(r,52),optical_target_r:i16(r,54),
      steer_gain_milli:(x>>>24)|((y&3)<<8),damping_milli:(y>>>2)&1023,
      stop_reason:(m>>>20)&15,control_flags:(m>>>24)&127,
      turn_degrees_x10:angle&4096?angle-8192:angle,junction_both_white:(y>>>12)&1,
      junction_prior_white:(y>>>13)&1,junction_departure_ready:(y>>>14)&1,
      junction_branch_accepted:(y>>>15)&1,junction_accepted_without_white:(y>>>16)&1,
      junction_accept_event:(y>>>17)&1,bend_phase:((m>>>31)&1)|(((y>>>31)&1)<<1)};
  }
  function decodeLogChunk(bytes) {
    const b = bytes instanceof Uint8Array ? bytes : new Uint8Array(bytes);
    if (b.length < 15) return { error: 'log chunk too short (' + b.length + ' bytes)' };
    const version=b[0]&15, src=b[0]>>4, len=b[1];
    if(version!==VERSION) return { error:'schema version '+version,src };
    if(src!==SRC.LOG_CHUNK) return { error:'source '+src+' is not a log chunk',src };
    if(b.length!==3+len) return { error:'length '+b.length+' != 3 + '+len,src };
    if(len<12) return { error:'log chunk payload '+len+' bytes, expected at least 12',src };
    const p=b.subarray(3), dataLen=p[9], partCount=p[7], partIndex=p[6], status=p[8];
    if(dataLen>49 || len!==12+dataLen) return { error:'log chunk data length mismatch',src };
    if(p[10]!==0 || p[11]!==0) return { error:'log chunk reserved bytes must be zero',src };
    if(!partCount || partCount>2 || partIndex>=partCount) return { error:'invalid log chunk part index/count',src };
    if(status && dataLen) return { error:'error log chunk contains data',src };
    return { src, srcName:SRC_NAME[src], seq:b[2], len, data:{
      dataset:p[0], requestId:p[1], rowIndex:u16(p,2), totalRows:u16(p,4), partIndex,
      partCount, status, dataLen, bytes:Array.from(p.subarray(12))
    }};
  }

  const DECODERS = { 1: decodeB1, 2: decodeB2Motion, 3: decodeLine, 4: decodeImu, 5: decodeB5,
                     6: decodeNav, 7: decodeSafety, 8: decodeMapping, 9: decodeMotion,
                     10: decodeLineEvent, 11: decodeNavEvent, 12: decodeSummary,
                     14: decodeDiagnostics };

  /* Decode one frame. Returns { src, srcName, seq, len, data } or { error }.
   * Unknown sources return data null (both originals did this). */
  function decodeFrame(bytes) {
    const b = bytes instanceof Uint8Array ? bytes : new Uint8Array(bytes);
    if (b.length < 3) return { error: 'too short (' + b.length + ' bytes)' };
    const version = b[0] & 0x0f, src = b[0] >> 4, len = b[1], seq = b[2];
    if (version !== VERSION) return { error: 'schema version ' + version, src };
    if (b.length !== 3 + len) return { error: 'length ' + b.length + ' != 3 + ' + len, src };
    const out = { src, srcName: name(SRC_NAME, src), seq, len };
    if (src === SRC.LOG_CHUNK) return decodeLogChunk(b);
    if (src === SRC.SOURCE15) {
      const d = decodeSource15(b.subarray(3));
      if (d.error) return { error: d.error, src };
      out.data = d; return out;
    }
    if (!(src in PAYLOAD)) { out.data = null; return out; }
    if (len !== PAYLOAD[src] && !(src === SRC.MOTION && B2_LENGTHS.includes(len)))
      return { error: 'payload ' + len + ' bytes, expected ' + PAYLOAD[src], src };
    out.data = DECODERS[src](b.subarray(3));
    if (out.data.error) return { error: out.data.error, src };
    return out;
  }

  /* Pico uptime carried by a decoded frame, in ms, or null when the frame
   * has none (sources 2-8 carry no uptime; source 1 has whole seconds;
   * source 9 may be run-relative when flags bit 7 is set). */
  function picoUptimeMs(f) {
    if (!f || !f.data) return null;
    const d = f.data;
    if (f.src === SRC.WIFI) return d.uptimeS * 1000;
    if (f.src >= 9 && f.src <= 14 && f.src !== SRC.LOG_CHUNK && Number.isFinite(d.uptimeMs)) return d.uptimeMs;
    if (f.src === 15 && Number.isFinite(d.uptimeMs)) return d.uptimeMs;
    return null;
  }
  function frameRunId(f) {
    if (!f || !f.data) return null;
    return (f.src === SRC.SDK_MOTION || f.src === SRC.SUMMARY ||
            (f.src === SRC.SOURCE15 && f.data.kind === 'imu-lift-safety')) ? f.data.runId : null;
  }

  function encodeCommand(code, id, arg) {
    const a = arg || 0;
    return new Uint8Array([(VERSION & 0x0f) | ((code & 0x0f) << 4), id & 0xff,
                           a & 0xff, (a >> 8) & 0xff]);
  }
  function configArg(param, value) { return ((param & 0x0f) << 12) | (value & 0x0fff); }
  /* WP CONFIG key 2 uses (mode << 8) | key. Telemetry key 1 keeps its
   * existing configArg encoding. */
  function driveModeArg(mode) {
    if (!Number.isInteger(mode) || mode < 0 || mode > 2) throw new Error('Drive mode must be 0, 1, or 2.');
    return (mode << 8) | CFG_DRIVE_MODE;
  }
  function getLogArg(dataset, row) { return ((dataset & 0x0f) << 12) | (row & 0x0fff); }

  function decodeAck(bytes) {
    const b = bytes instanceof Uint8Array ? bytes : new Uint8Array(bytes);
    if (b.length !== 4) return { error: 'ack length ' + b.length };
    if ((b[0] & 15) !== VERSION) return { error: 'ack schema version ' + (b[0] & 15) };
    const code = b[0] >> 4;
    return { code, codeName: CMD_NAME[code], id: b[1], result: b[2],
             resultName: name(ACK_NAME, b[2]), accepted: b[2] === 0,
             state: b[3] & 7, stateName: name(NAV_STATE, b[3] & 7) };
  }

  /* Firmware family from the sources observed since the last connect/boot.
   * Manual override wins. Returns 'rtos' | 'sdk' | 'mixed' | 'unknown'. */
  /* Inference for the RTOS Buddy4 bench build, which only sends shared
   * sources (1, 2, 4, 8): SDK v2.3 sends source 9/10 every 50 ms whenever
   * telemetry forwarding is on, and the v2.3 Buddy4 add-on's sources 2/4 are
   * gated by that same switch. So Buddy2/Buddy4 frames for >= 3 s with no
   * SDK-only source means RTOS ('rtos-inferred'). firstSeen maps source ->
   * time first seen (ms) in the current observation window. */
  const INFER_MS = 3000;
  function detectProfile(seenSources, firstSeen, nowMs) {
    const seen = Array.from(seenSources || []).map(Number);
    const rtos = seen.some((x) => SRC_FAMILY[x] === 'rtos');
    const sdk = seen.some((x) => SRC_FAMILY[x] === 'sdk');
    if (rtos && sdk) return 'mixed';
    if (sdk) return 'sdk';
    if (rtos) return 'rtos';
    if (firstSeen && Number.isFinite(nowMs)) {
      const get = (k) => (firstSeen instanceof Map ? firstSeen.get(k) : firstSeen[k]);
      const b4 = [SRC.MOTION, SRC.IMU].map(get).filter(Number.isFinite);
      if (b4.length && nowMs - Math.min(...b4) >= INFER_MS) return 'rtos-inferred';
    }
    return 'unknown';
  }
  function effectiveProfile(detected, override) {
    if (override === 'rtos' || override === 'sdk') return override;
    return detected === 'rtos-inferred' ? 'rtos' : detected;
  }

  /* Sequence tracker for the delivery test (NFR-WIFI-01). */
  function SeqTracker() { this.reset(); }
  SeqTracker.prototype.reset = function () { this.per = {}; };
  SeqTracker.prototype.add = function (src, seq) {
    let t = this.per[src];
    if (!t) { t = this.per[src] = { last: seq, received: 1, missing: 0, dup: 0 }; return; }
    const gap = (seq - t.last) & 0xff;
    if (gap === 0) { t.dup++; return; }
    if (gap < 128) t.missing += gap - 1;      /* else: late or restarted, ignore */
    t.received++; t.last = seq;
  };
  SeqTracker.prototype.summary = function () {
    let r = 0, m = 0;
    for (const k in this.per) { r += this.per[k].received; m += this.per[k].missing; }
    return { received: r, missing: m, pct: r + m ? (100 * r) / (r + m) : 100 };
  };
  function connectivityVerdict(per, durationS, maxGap, beats, finished) {
    const sources = Object.keys(per).filter((k) => Number(k) !== SRC.BARCODE_MAP);
    const complete = finished && durationS >= 300;
    const deliveryGood = sources.includes(String(SRC.WIFI)) && sources.every((k) => {
      const t = per[k];
      return t.received > 0 && 100 * t.received / (t.received + t.missing) >= 95;
    });
    const heartbeatGood = beats > 1 && maxGap <= 2;
    return { complete, deliveryGood, heartbeatGood,
             deliveryPass: complete && deliveryGood, heartbeatPass: complete && heartbeatGood };
  }

  const api = { VERSION, SRC, SRC_NAME, SRC_FAMILY, MAX_FRAME, CMD, CMD_NAME, MOTION_CMDS,
                ACK_NAME, CFG_TELEMETRY, CFG_DRIVE_MODE, CFG_CALIBRATION, DRIVE_MODES, NAV_STATE, NET_STATE, EVENT_NAME, PROFILE_NAME,
                SDK_MODE, SDK_OWNER, SDK_MODE_OFF, SDK_MODE_FAULT, LOG_DATASETS, PAYLOAD,
                decodeFrame, encodeCommand, configArg, driveModeArg, calibrationArg, getLogArg, decodeAck, SeqTracker,
                le32, u, s, u16, u32, i16, MAP_MOVEMENTS, mappingWord, encodeMapping,
                mappedMovement, mappingReadbackStatus, mappingTimedOut, connectivityVerdict,
                decodeB1, decodeB2Motion, decodeImu, decodeLine, decodeB5, decodeNav,
                decodeSafety, decodeMapping, decodeMotion, decodeLineEvent, decodeNavEvent,
                decodeSummary, decodeDiagnostics, decodeLogChunk, decodeRampAssist, decodeBarcodePeak,
                decodeEncoderFault, decodeBarcodeCapture, decodeNetworkStop, decodeImuLiftSafety, decodeImuPeakEvidence, decodeDriveModeStatus, decodeCalibrationStatus,
                decodeSource15, decodeStoredLine, picoUptimeMs, frameRunId,
                detectProfile, effectiveProfile,
                decodeB5Source15, decodeB5MissionStatus, decodeB5WidthRecord, decodeB5WidthSummary,
                decodeB5WidthResult, decodeB5WidthRetry, decodeB5DirectionRecord, decodeB5DirectionSummary,
                decodeB5Event, B5Tracker, B5M_STAGE, B5M_REASON, B5M_BASIS };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.IS28 = api;
})(typeof self !== 'undefined' ? self : this);
