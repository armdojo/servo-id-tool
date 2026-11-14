/* ArmDojo Servo ID tool — protocol, bus and numbering flow. No browser APIs here (runs in Node for the tests).
 * Same flow as our native Mac and Android servo tools, verified there on real STS3215 servos; HD-1910 verified on a real servo 2026-11-15.
 * Packet: FF FF ID LEN INSTR PARAMS… CHK,  LEN = params + 2,  CHK = ~(ID + LEN + INSTR + params) & 0xFF.
 * Link interface: { baud, setBaud(b), flush(), write(Uint8Array), read(arr, want, deadlineMs) } (async where it waits). */
(function (root) {
  'use strict';
  const BROADCAST = 0xFE, PING = 0x01, READ = 0x02, WRITE = 0x03;
  const REG = { FW: 0, MODEL: 3, ID: 5, BAUD: 6, MAX_ANGLE: 11, MAX_VOLT: 14, UNLOAD: 19, MODE: 33, TORQUE: 40, ACC: 41, GOAL: 42,
    TORQUE_LIMIT: 48, LOCK: 55, LOCK_SCS: 48, POS: 56, VOLT: 62, TEMP: 63 };
  /** Unload-condition bits (register 19, STS family): the servo cuts torque when one trips. */
  const UNLOAD = { VOLTAGE: 1, ENCODER: 2, HEAT: 4, CURRENT: 8, OVERLOAD: 32 };
  /** Microduck joint IDs (official runtime duck-control/src/model.rs, used by the community replicas). */
  const MICRODUCK_IDS = [10, 11, 12, 13, 14, 20, 21, 22, 23, 24, 30, 31, 32, 33, 34];
  const SCAN_BAUDS = [1000000, 500000, 250000, 115200, 57600, 38400, 128000, 76800];
  const BAUD_INDEX = { 1000000: 0, 500000: 1, 250000: 2, 128000: 3, 115200: 4, 76800: 5, 57600: 6, 38400: 7 };
  const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
  const now = () => (typeof performance !== 'undefined' ? performance.now() : Date.now());

  class BusError extends Error {
    /** kind: 'io' (board gone), 'collision' (garbled / two servos), 'noreply', 'failed' */
    constructor(kind, message) { super(message); this.kind = kind; }
  }

  function checksum(b, from, to) { let s = 0; for (let i = from; i < to; i++) s += b[i]; return (~s) & 0xFF; }

  function packet(id, instr, params) {
    params = params || [];
    const p = new Uint8Array(params.length + 6);
    p[0] = 0xFF; p[1] = 0xFF; p[2] = id; p[3] = params.length + 2; p[4] = instr;
    p.set(params, 5);
    p[p.length - 1] = checksum(p, 2, p.length - 1);
    return p;
  }

  const hex = (b) => Array.from(b, (x) => x.toString(16).padStart(2, '0').toUpperCase()).join(' ');
  const startsWith = (buf, pkt) => buf.length >= pkt.length && pkt.every((v, i) => buf[i] === v);
  const isPrefixOf = (buf, pkt) => buf.length > 0 && buf.length < pkt.length && buf.every((v, i) => pkt[i] === v);

  class Bus {
    constructor(link) { this.link = link; this.timeout = 25; this.lastReplyId = -1; }
    get baud() { return this.link.baud; }
    async setBaud(b) { await this.link.setBaud(b); }

    /** One request and its reply; null on timeout. opt.accept: reply IDs allowed (default the addressed ID). */
    async transact(id, instr, params, opt) {
      opt = opt || {};
      const pkt = packet(id, instr, params);
      this.link.flush();
      await this.link.write(pkt);
      const deadline = now() + (opt.timeout || this.timeout);
      const buf = [];
      await this.link.read(buf, 6, deadline);
      if (isPrefixOf(buf, pkt)) await this.link.read(buf, pkt.length, deadline);   // a board that echoes TX
      if (startsWith(buf, pkt)) { buf.splice(0, pkt.length); await this.link.read(buf, 6, deadline); }
      if (buf.length === 0) return null;
      let h = -1;
      for (let i = 0; i + 1 < buf.length; i++) if (buf[i] === 0xFF && buf[i + 1] === 0xFF) { h = i; break; }
      if (h < 0) throw new BusError('collision', 'garbled reply ' + hex(buf));
      buf.splice(0, h);
      while (buf.length >= 3 && buf[2] === 0xFF) buf.shift();
      await this.link.read(buf, 4, deadline);
      if (buf.length < 4 || buf[3] < 2) throw new BusError('collision', 'short reply ' + hex(buf));
      const len = buf[3];
      await this.link.read(buf, 4 + len, deadline + 5);
      if (buf.length < 4 + len) throw new BusError('collision', 'incomplete reply ' + hex(buf));
      const body = buf.slice(2, 3 + len);
      if (checksum(body, 0, body.length) !== buf[3 + len]) throw new BusError('collision', 'bad checksum ' + hex(buf));
      const rid = body[0];
      if (id !== BROADCAST && !(opt.accept || [id]).includes(rid)) throw new BusError('collision', `asked ID ${id}, ID ${rid} answered`);
      // Anything after a good packet means a second servo is talking.
      const extra = buf.slice(4 + len);
      if (extra.length === 0) { await sleep(id === BROADCAST ? 6 : 3); await this.link.read(extra, 1, now()); }
      if (extra.length > 0) throw new BusError('collision', 'more than one servo answered');
      this.lastReplyId = rid;
      return { error: body[2], params: body.slice(3) };
    }

    async ping(id, timeout) { return (await this.transact(id, PING, [], { timeout })) !== null; }

    /** Ping where a garbled answer still counts as "someone is there". */
    async somebodyAt(id) {
      try { return await this.ping(id, 20); } catch (e) { if (e.kind === 'collision') return true; throw e; }
    }

    async read(id, addr, n) {
      const r = await this.transact(id, READ, [addr, n]);
      if (!r) throw new BusError('noreply', `ID ${id} did not answer (read ${addr})`);
      if (r.params.length !== n) throw new BusError('collision', `ID ${id} returned the wrong length`);
      return r.params;
    }

    async write(id, addr, data, opt) {
      const r = await this.transact(id, WRITE, [addr].concat(Array.from(data)), opt);
      if (!r) throw new BusError('noreply', `ID ${id} did not confirm the write (register ${addr})`);
      return r;
    }

    async read16(id, addr) { const b = await this.read(id, addr, 2); return b[0] | (b[1] << 8); }

    /** The single servo answering a broadcast ping at the current baud, or null. */
    async broadcastPing() { return (await this.transact(BROADCAST, PING, [], { timeout: 30 })) ? this.lastReplyId : null; }

    /** Try every baud rate; {id, baud} of the one servo on the bus, or null. */
    async findSingle(bauds) {
      for (const b of bauds || SCAN_BAUDS) {
        await this.setBaud(b);
        const id = await this.broadcastPing();
        if (id !== null) return { id, baud: b };
      }
      return null;
    }

    async scan(onProgress, timeout) {
      const found = [], garbled = [];
      for (let id = 0; id <= 253; id++) {
        try { if (await this.ping(id, timeout || 12)) found.push(id); }
        catch (e) { if (e.kind === 'collision') garbled.push(id); else throw e; }
        if (onProgress && id % 8 === 0) onProgress(id / 253);
      }
      return { found, garbled };
    }

    async identify(id) {
      const s = { id, baud: this.baud, modelRaw: 0, model: '?', family: 'sts', known: false, fw: '', maxVolt: null, voltage: null, temp: null, unload: null, mode: null };
      const fw = await this.read(id, REG.FW, 2);
      s.fw = fw[0] + '.' + fw[1];
      s.modelRaw = await this.read16(id, REG.MODEL);
      s.maxVolt = (await this.read(id, REG.MAX_VOLT, 1))[0];
      if (s.modelRaw === 777) { s.model = 'STS3215 ' + (s.maxVolt >= 120 ? '(12 V)' : '(7.4 V)'); s.known = true; }
      else if (s.modelRaw === 2825) { s.model = 'STS3250 (12 V)'; s.known = true; }
      else if (s.modelRaw === 0x0405) { s.model = 'SCS0009'; s.family = 'scs'; s.known = true; }
      else if (s.modelRaw === 7946) { s.model = 'HD-1910'; s.known = true; }   // HD-1910-C001, fw 3.46, read on a real servo 2026-11-15
      else {
        s.model = 'Model ' + s.modelRaw;
        // STS: max angle 4095 little-endian (FF 0F). SCS: 1023 big-endian (03 FF).
        const a = await this.read(id, REG.MAX_ANGLE, 2);
        s.family = (a[1] === 0xFF && a[0] <= 0x0F) ? 'scs' : 'sts';
      }
      s.voltage = (await this.read(id, REG.VOLT, 1))[0] / 10;
      s.temp = (await this.read(id, REG.TEMP, 1))[0];
      if (s.family === 'sts') {
        s.unload = (await this.read(id, REG.UNLOAD, 1))[0];
        s.mode = (await this.read(id, REG.MODE, 1))[0];
      }
      return s;
    }

    lockAddr(s) { return s.family === 'scs' ? REG.LOCK_SCS : REG.LOCK; }

    /** Unlock EEPROM, write ID, lock on the new ID, read back. Throws if anything does not check out. */
    async changeId(s, next) {
      if (!(next >= 0 && next <= 253)) throw new BusError('failed', 'ID must be 0–253');
      const old = s.id, lock = this.lockAddr(s);
      if (next !== old) {
        if (await this.somebodyAt(next)) throw new BusError('failed', `ID ${next} is already used by another servo on the bus`);
        await this.write(old, lock, [0]);
        // The ack comes from the OLD id, 2–40 ms later (flash write).
        const ack = await this.transact(old, WRITE, [REG.ID, next], { timeout: 300, accept: [old, next] });
        if (!ack && !(await this.ping(next, 60))) throw new BusError('noreply', 'no answer to the ID write');
        await sleep(10);
      }
      await this.write(next, lock, [1]);
      const got = (await this.read(next, REG.ID, 1))[0];
      if (got !== next) throw new BusError('failed', `read back ID ${got}, not ${next}`);
      if (s.family === 'sts' && (await this.read(next, lock, 1))[0] !== 1) throw new BusError('failed', 'EEPROM did not lock: the ID may be lost at power-off');
      if (next !== old && await this.somebodyAt(old)) throw new BusError('failed', `ID ${old} still answers: another servo with the same ID is on the bus`);
      s.id = next;
    }

    /** Switch on unload-condition bits (keeping the ones already on) so they survive power-off: unlock EEPROM, write,
     *  lock, read back. STS family only. Returns the new register value. */
    async setUnload(s, bits) {
      if (s.family !== 'sts') throw new BusError('failed', 'protection settings are only for STS-family servos');
      const cur = (await this.read(s.id, REG.UNLOAD, 1))[0], want = cur | bits;
      if (want !== cur) {
        await this.write(s.id, REG.LOCK, [0]);
        await this.write(s.id, REG.UNLOAD, [want], { timeout: 300 });   // EEPROM write: the ack can take a few ms
        await sleep(10);
        await this.write(s.id, REG.LOCK, [1]);
      }
      const got = (await this.read(s.id, REG.UNLOAD, 1))[0];
      if (got !== want) throw new BusError('failed', `protection register reads ${got}, not ${want}`);
      if ((await this.read(s.id, REG.LOCK, 1))[0] !== 1) throw new BusError('failed', 'EEPROM did not lock: the protection may be lost at power-off');
      s.unload = got;
      return got;
    }

    async setBaudRegister(s, b) {
      const idx = BAUD_INDEX[b];
      if (idx === undefined || b === s.baud) return;
      const lock = this.lockAddr(s);
      await this.write(s.id, lock, [0]);
      await this.link.write(packet(s.id, WRITE, [REG.BAUD, idx]));   // the ack comes at the old baud; ignore it
      await sleep(40);
      await this.setBaud(b);
      await this.write(s.id, lock, [1]);
      if (!(await this.ping(s.id, 60))) throw new BusError('failed', 'servo not found after changing the baud rate');
      s.baud = b;
    }

    /** Turn one STS servo to the middle (2048) slowly at 30 % torque and keep holding it. Returns the final position.
     *  The goal is stepped here (≈ 35°/s) instead of relying on the speed register: HD-1910s ship in mode 4 (position PD),
     *  which may ignore it and would jump straight to the goal. */
    async center(id, onPos) {
      const v = (await this.read(id, REG.VOLT, 1))[0] / 10;
      if (v < 6) throw new BusError('failed', 'power:' + v.toFixed(1));
      const pos = await this.read16(id, REG.POS) & 0x0FFF;
      const goal = (p, speed) => [10, p & 0xFF, p >> 8, 0, 0, speed & 0xFF, speed >> 8];
      await this.write(id, REG.ACC, goal(pos, 400));          // hold where it is first: the goal register may hold 0
      await this.write(id, REG.TORQUE_LIMIT, [300 & 0xFF, 300 >> 8]);
      await this.write(id, REG.TORQUE, [1]);
      const t0 = now();
      let g = pos, p = pos;
      while (now() - t0 < 12000) {
        if (g !== 2048) {
          g += Math.sign(2048 - g) * Math.min(Math.abs(2048 - g), 20);   // 20 steps per 50 ms ≈ 35°/s
          await this.write(id, REG.GOAL, [g & 0xFF, g >> 8]);
        }
        await sleep(50);
        p = await this.read16(id, REG.POS) & 0x0FFF;
        if (onPos) onPos(p);
        if (g === 2048 && Math.abs(p - 2048) <= 8) break;
      }
      return p;
    }

    async release(id) { await this.write(id, REG.TORQUE, [0]); }
  }

  /** One-at-a-time numbering: plug a servo in, it gets the next ID, unplug, repeat. Status objects carry i18n keys.
   *  cfg.ids: the IDs to hand out in order (default next…end); cfg.unloadBits: protection bits to switch on in each servo. */
  class Numberer {
    constructor(bus, cfg, onStatus, onNumbered) {
      this.bus = bus;
      this.cfg = Object.assign({ next: 1, end: 253, ids: null, unloadBits: 0, force1M: true, done: new Set(), lastNumbered: null }, cfg);
      if (!this.cfg.ids) { this.cfg.ids = []; for (let i = this.cfg.next; i <= this.cfg.end; i++) this.cfg.ids.push(i); }
      this.queue = this.cfg.ids.slice();
      this.onStatus = onStatus || (() => {});
      this.onNumbered = onNumbered || (() => {});
      this.stopped = false;
      this.last = '';
    }
    stop() { this.stopped = true; }
    report(s) { const k = JSON.stringify(s); if (k !== this.last) { this.last = k; this.onStatus(s); } }

    async run() {
      const c = this.cfg, b = this.bus;
      let state = c.lastNumbered ? { removal: c.lastNumbered } : { removal: null };
      let misses = 0;
      const last = c.ids.length ? c.ids[c.ids.length - 1] : c.end;
      if (c.lastNumbered && this.queue.length) this.report({ phase: 'waiting', big: String(this.queue[0]), key: 'remove_first', args: { id: c.lastNumbered.id, n: this.queue[0] } });
      while (!this.stopped) {
        try {
          if (!state.removal) {
            if (!this.queue.length) { this.report({ phase: 'finished', big: '✓', key: 'finished', args: { end: last } }); return; }
            this.report({ phase: 'waiting', big: String(this.queue[0]), key: 'plug_next', args: { n: this.queue[0] } });
            const f = await b.findSingle();
            if (this.stopped) return;
            if (!f) { await sleep(150); continue; }
            const s = await b.identify(f.id);
            if (c.done.has(f.id) && f.id !== 1 && f.id !== this.queue[0]) {
              this.report({ phase: 'warning', big: String(f.id), key: 'already', args: { id: f.id }, servo: s });
              state = { removal: { id: f.id, baud: s.baud } }; misses = 0;
              continue;
            }
            const target = this.queue[0], old = s.id;
            this.report({ phase: 'working', big: String(target), key: 'writing', args: { n: target }, servo: s });
            await b.changeId(s, target);
            if (c.force1M && s.baud !== 1000000) await b.setBaudRegister(s, 1000000);
            let protect = null;   // null: not asked · true: switched on · false: not available for this model
            if (c.unloadBits) { protect = s.family === 'sts'; if (protect) await b.setUnload(s, c.unloadBits); }
            const v = await b.identify(target);   // fresh read-back
            v.baud = s.baud; v.protect = protect;
            if (protect && (v.unload & c.unloadBits) !== c.unloadBits) throw new BusError('failed', 'protection did not read back');
            c.done.add(target); this.queue.shift(); c.next = this.queue.length ? this.queue[0] : target + 1;
            this.onNumbered(old, v, this.queue.length ? this.queue[0] : null);
            this.report({ phase: 'success', big: String(target), key: v.known ? 'done' : 'done_new_model', args: { n: target, old }, servo: v });
            state = { removal: { id: target, baud: v.baud } }; misses = 0;
          } else {
            await b.setBaud(state.removal.baud);
            if (await b.somebodyAt(state.removal.id)) misses = 0;
            else if (++misses >= 3) { state = { removal: null }; continue; }
            await sleep(120);
          }
        } catch (e) {
          if (this.stopped) return;
          if (e.kind === 'io') { this.report({ phase: 'error', big: 'USB', key: 'io_lost', args: {} }); this.stopped = true; return; }
          if (e.kind === 'collision') this.report({ phase: 'error', big: '!', key: 'collision', args: {} });
          else this.report({ phase: 'error', big: '!', key: 'error', args: { msg: e.message } });
          await sleep(500);
        }
      }
    }
  }

  function compressIds(ids) {
    const s = Array.from(ids).sort((a, b) => a - b), parts = [];
    for (let i = 0; i < s.length;) {
      let j = i;
      while (j + 1 < s.length && s[j + 1] === s[j] + 1) j++;
      parts.push(j > i ? `${s[i]}–${s[j]}` : `${s[i]}`);
      i = j + 1;
    }
    return parts.join(', ');
  }

  const api = { BROADCAST, PING, READ, WRITE, REG, UNLOAD, MICRODUCK_IDS, SCAN_BAUDS, BAUD_INDEX, BusError, checksum, packet, hex, Bus, Numberer, compressIds, sleep, now };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.ArmDojoServoCore = api;
})(typeof self !== 'undefined' ? self : this);
