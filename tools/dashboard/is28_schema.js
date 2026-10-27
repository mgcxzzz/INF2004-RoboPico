/*
 * is28_schema.js -- decode the IS28 team telemetry frames and build
 * supervisory commands, exactly as app_program/system/sys_tlm.h defines
 * them (bit positions, widths, signs). Used by index.html in the browser
 * and by testing/subsystem1/host/check_dashboard.mjs in Node.
 *
 * Rules (sys_tlm.h): 3-byte header [version | source << 4, payload length,
 * sequence], then little-endian 32-bit words; signed fields are two's
 * complement in their own width. Keep this file in step with sys_tlm.h.
 *
 * Group IS28, Buddy 1.
 */
(function (root) {
  'use strict';

  const VERSION = 2;
  const SRC = { WIFI: 1, MOTION: 2, LINE: 3, IMU: 4, OBSTACLE: 5, NAV: 6, SAFETY: 7,
                BARCODE_MAP: 8 };
  const SRC_NAME = ['?', 'WiFi (B1)', 'Motion (B2)', 'Line (B3)', 'IMU (B4)',
                    'Obstacle (B5)', 'Navigation', 'Safety', 'Barcode mapping'];

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

  /* Commands (sys_tlm.h, "Supervisory commands"). */
  const CMD = { START: 1, PAUSE: 2, RESUME: 3, STOP: 4, RESET: 5, ESTOP: 6, CONFIG: 7,
                PING: 8, BARCODE_MAP: 9, BARCODE_GET: 10, STEER: 12, DRIVE: 13, TURN: 14 };
  const CMD_NAME = ['none', 'START', 'PAUSE', 'RESUME', 'STOP', 'RESET', 'ESTOP',
                    'CONFIG', 'PING', 'BARCODE_MAP', 'BARCODE_GET', 'code11', 'STEER', 'DRIVE',
                    'TURN', 'code15'];
  const ACK_NAME = ['accepted', 'rejected: steering/navigation not allowed (FR-WIFI-03)',
                    'rejected: unknown command', 'rejected: bad format',
                    'rejected: queue full', 'rejected: unsupported setting'];
  const CFG_TELEMETRY = 1;

  /* Payload length per source; a frame of any other length is refused. */
  const PAYLOAD = { 1: 12, 3: 12, 4: 16, 5: 16, 6: 6, 7: 12, 8: 5 };
  const MAP_MOVEMENTS = ['Forward', 'U-turn', 'Left', 'Right'];

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

  function u(word, shift, bits) {
    return bits >= 32 ? (word >>> shift) >>> 0 : (word >>> shift) & ((1 << bits) - 1);
  }
  function s(word, shift, bits) {
    const raw = u(word, shift, bits);
    const sign = 1 << (bits - 1);
    return (raw ^ sign) - sign;
  }
  function le32(b, i) {
    return (b[i] | (b[i + 1] << 8) | (b[i + 2] << 16) | (b[i + 3] << 24)) >>> 0;
  }
  function name(list, i) { return i < list.length ? list[i] : ('#' + i); }

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

  const DECODERS = { 1: decodeB1, 3: decodeLine, 4: decodeImu, 5: decodeB5,
                     6: decodeNav, 7: decodeSafety, 8: decodeMapping };

  /* Decode one frame. Returns { src, srcName, seq, data } or { error }. */
  function decodeFrame(bytes) {
    const b = bytes instanceof Uint8Array ? bytes : new Uint8Array(bytes);
    if (b.length < 3) return { error: 'too short (' + b.length + ' bytes)' };
    const version = b[0] & 0x0f, src = b[0] >> 4, len = b[1], seq = b[2];
    if (version !== VERSION) return { error: 'schema version ' + version, src };
    if (b.length !== 3 + len) return { error: 'length ' + b.length + ' != 3 + ' + len, src };
    const out = { src, srcName: name(SRC_NAME, src), seq, len };
    if (!(src in PAYLOAD)) { out.data = null; return out; }        /* not defined yet */
    if (len !== PAYLOAD[src]) return { error: 'payload ' + len + ' bytes, expected ' + PAYLOAD[src], src };
    out.data = DECODERS[src](b.subarray(3));
    if (out.data.error) return { error: out.data.error, src };
    return out;
  }

  function encodeCommand(code, id, arg) {
    const a = arg || 0;
    return new Uint8Array([(VERSION & 0x0f) | ((code & 0x0f) << 4), id & 0xff,
                           a & 0xff, (a >> 8) & 0xff]);
  }

  function configArg(param, value) { return ((param & 0x0f) << 12) | (value & 0x0fff); }

  function decodeAck(bytes) {
    const b = bytes instanceof Uint8Array ? bytes : new Uint8Array(bytes);
    if (b.length !== 4) return { error: 'ack length ' + b.length };
    if ((b[0] & 15) !== VERSION) return { error: 'ack schema version ' + (b[0] & 15) };
    const code = b[0] >> 4;
    return { code, codeName: CMD_NAME[code], id: b[1], result: b[2],
             resultName: name(ACK_NAME, b[2]), accepted: b[2] === 0,
             state: b[3] & 7, stateName: name(NAV_STATE, b[3] & 7) };
  }

  /* Sequence tracker for the delivery test (NFR-WIFI-01): counts frames
   * received and missing per source from the wrapping 8-bit sequence. */
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

  /* A stopped-short run or a silent beginning/end must never earn PASS.
   * Delivery is a sequence-gap estimate, assessed per observed source.
   * Source 8 is on-demand configuration, not periodic telemetry. */
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

  const api = { VERSION, SRC, SRC_NAME, CMD, CMD_NAME, ACK_NAME, CFG_TELEMETRY,
                NAV_STATE, NET_STATE, decodeFrame, encodeCommand, configArg,
                decodeAck, SeqTracker, le32, u, s, MAP_MOVEMENTS, mappingWord,
                encodeMapping, mappedMovement, connectivityVerdict };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.IS28 = api;
})(typeof self !== 'undefined' ? self : this);
