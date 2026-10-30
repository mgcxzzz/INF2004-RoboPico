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
                    'Ramp assist / barcode peak'];
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
  /* Source 15 carries TWO layouts in SDK v2.3 integration builds; dispatch is by
   * validated payload length and subtype, never by source number alone:
   *  - ramp assist v0.1/v0.2 (+ Buddy4 estimator marker): payload 28 bytes
   *    (ramp_assist.h ra_tlm_pack; flags bits4-6 detector, bit7 estimator v0.2);
   *  - barcode peak decoder v0.1: payload 24 bytes, payload[4] = 0xB1
   *    (barcode_peak.h bp_tlm_pack; flags bit7 = combined Buddy1-4 build).
   * Anything else is rejected as an unknown layout. */
  const RA_EVENT = ['periodic', 'activate', 'end', 'stall push', 'floor/hold end'];   /* 3 = v0.4 stall push started, 4 = v0.5 tilt floor / v0.6 speed hold ended */
  const RA_END = ['none', 'level', 'descent', 'invalid', 'stale', 'timeout', 'not following', 'paused', 'stopped'];
  const BP_REASON = ['ok', 'too few bars', 'bar height unclear', 'wide bar count', 'unit out of range',
                     'space unclear', 'wide character gap', 'not a *X* frame', 'ambiguous', 'no wide bars',
                     'run start'];
  function i8(p, i) { return p[i] & 0x80 ? p[i] - 256 : p[i]; }
  function decodeRampAssist(p) {
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
  function decodeSource15(p) {
    if (p.length === 28) return decodeRampAssist(p);
    if (p.length === 24 && p[4] === 0xB1) return decodeBarcodePeak(p);
    return { error: 'unknown source-15 layout (payload ' + p.length + ' bytes' + (p.length > 4 ? ', byte4 0x' + p[4].toString(16) : '') + ')' };
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
    return null;
  }
  function frameRunId(f) {
    if (!f || !f.data) return null;
    return (f.src === SRC.SDK_MOTION || f.src === SRC.SUMMARY) ? f.data.runId : null;
  }

  function encodeCommand(code, id, arg) {
    const a = arg || 0;
    return new Uint8Array([(VERSION & 0x0f) | ((code & 0x0f) << 4), id & 0xff,
                           a & 0xff, (a >> 8) & 0xff]);
  }
  function configArg(param, value) { return ((param & 0x0f) << 12) | (value & 0x0fff); }
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
                ACK_NAME, CFG_TELEMETRY, NAV_STATE, NET_STATE, EVENT_NAME, PROFILE_NAME,
                SDK_MODE, SDK_OWNER, SDK_MODE_OFF, SDK_MODE_FAULT, LOG_DATASETS, PAYLOAD,
                decodeFrame, encodeCommand, configArg, getLogArg, decodeAck, SeqTracker,
                le32, u, s, u16, u32, i16, MAP_MOVEMENTS, mappingWord, encodeMapping,
                mappedMovement, mappingReadbackStatus, mappingTimedOut, connectivityVerdict,
                decodeB1, decodeB2Motion, decodeImu, decodeLine, decodeB5, decodeNav,
                decodeSafety, decodeMapping, decodeMotion, decodeLineEvent, decodeNavEvent,
                decodeSummary, decodeDiagnostics, decodeLogChunk, decodeRampAssist, decodeBarcodePeak,
                decodeSource15, picoUptimeMs, frameRunId,
                detectProfile, effectiveProfile };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.IS28 = api;
})(typeof self !== 'undefined' ? self : this);
