/*! ArmDojo Servo ID tool v3 — https://armdojo.com/pages/servo-id-tool */
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

/* Simulated servos behind a Link: parses what the bus sends and answers like STS3215s (or HD-1910s) do.
 * EEPROM (addresses < 40) keeps a write over a power cycle only if register 55 (lock) was 0 when it was written.
 * Used by the Node tests and by the page's demo mode (?servo-demo=1), where servos get "plugged in" one by one. */
(function (root) {
  'use strict';
  const core = (typeof module !== 'undefined' && module.exports) ? require('./core.js') : root.ArmDojoServoCore;
  const { checksum, packet, BAUD_INDEX, REG } = core;

  class MockServo {
    constructor(id, opt) {
      opt = opt || {};
      this.baud = opt.baud || 1000000;
      this.mem = new Array(80).fill(0);
      const model = opt.model || 777;
      Object.assign(this.mem, { 0: 3, 1: 10, 3: model & 0xFF, 4: model >> 8, 5: id, 6: BAUD_INDEX[this.baud] || 0,
        11: 0xFF, 12: 0x0F, 14: opt.maxVolt || 140, 19: opt.unload != null ? opt.unload : 4, 33: opt.mode || 0,
        48: 1000 & 0xFF, 49: 1000 >> 8, 55: 1, 62: opt.volt || 121, 63: 31 });
      this.rom = this.mem.slice(0, 40);
      this.pos = opt.pos != null ? opt.pos : 1500;
      this.answers = true; this.replyAs = null; this.corrupt = false; this.noise = null;
    }
    get id() { return this.mem[REG.ID]; }
    /** Power off and on: EEPROM falls back to what was saved, torque off, lock on. */
    powerCycle() {
      this.rom.forEach((v, a) => { this.mem[a] = v; });
      this.mem[REG.TORQUE] = 0; this.mem[REG.LOCK] = 1;
      const e = Object.entries(BAUD_INDEX).find(([, i]) => i === this.mem[REG.BAUD]);
      if (e) this.baud = Number(e[0]);
    }
    get torque() { return this.mem[REG.TORQUE] !== 0; }
    w(a) { return this.mem[a] | (this.mem[a + 1] << 8); }
    advance(dt) {
      if (!this.torque) return;
      const goal = this.w(REG.GOAL), v = (this.w(REG.GOAL + 4) || 3400) * dt, d = goal - this.pos;
      this.pos = Math.abs(d) <= v ? goal : this.pos + Math.sign(d) * v;
    }
  }

  class MockLink {
    constructor(opt) {
      opt = opt || {};
      this.echo = !!opt.echo; this.servos = []; this.sent = []; this.rx = []; this.baud = 1000000;
      this.name = 'demo'; this.lastT = Date.now();
    }
    add(s) { this.servos.push(s); return s; }
    servo(id) { return this.servos.find((s) => s.id === id); }
    async setBaud(b) { this.baud = b; this.rx.length = 0; }
    flush() { this.rx.length = 0; }
    async close() {}
    tick() { const t = Date.now(); const dt = (t - this.lastT) / 1000; this.lastT = t; this.servos.forEach((s) => s.advance(dt)); }
    async read(out, want, deadline) { this.tick(); out.push(...this.rx); this.rx.length = 0; }   // replies are queued at once
    live(id) { return this.servos.filter((s) => s.answers && s.baud === this.baud && (id === 0xFE || s.id === id)); }
    reply(s, params, err) {
      if (s.noise) this.rx.push(...s.noise);
      const p = packet(s.replyAs != null ? s.replyAs : s.id, err || 0, params || []);
      if (s.corrupt) p[p.length - 1] = (p[p.length - 1] + 1) & 0xFF;
      this.rx.push(...p);
    }
    readMem(s, a, n) { const p = Math.round(s.pos); s.mem[56] = p & 0xFF; s.mem[57] = p >> 8; return s.mem.slice(a, a + n); }
    writeMem(s, a, d) {
      const unlocked = s.mem[REG.LOCK] === 0;
      d.forEach((v, k) => { if (a + k < s.mem.length) s.mem[a + k] = v; if (unlocked && a + k < 40) s.rom[a + k] = v; });
      if (a <= REG.BAUD && a + d.length > REG.BAUD) {
        const e = Object.entries(BAUD_INDEX).find(([, i]) => i === s.mem[REG.BAUD]);
        if (e) s.baud = Number(e[0]);
      }
    }
    async write(data) {
      this.sent.push(Array.from(data));
      if (this.echo) this.rx.push(...data);
      this.tick();
      for (let i = 0; i + 6 <= data.length;) {
        if (data[i] !== 0xFF || data[i + 1] !== 0xFF) { i++; continue; }
        const end = i + 4 + data[i + 3];
        if (end > data.length) break;
        if (checksum(data, i + 2, end - 1) === data[end - 1]) this.handle(data[i + 2], data[i + 4], Array.from(data.slice(i + 5, end - 1)));
        i = end;
      }
    }
    handle(id, instr, p) {
      if (instr === 1) this.live(id).forEach((s) => this.reply(s));
      else if (instr === 2) this.live(id).forEach((s) => this.reply(s, this.readMem(s, p[0], p[1])));
      else if (instr === 3) this.live(id).forEach((s) => { if (id !== 0xFE) this.reply(s); this.writeMem(s, p[0], p.slice(1)); s.written = true; });
    }
  }

  /** Demo bench: a factory servo (ID 1, random baud) appears; once it has a new ID it is "unplugged" and the next one appears. */
  class DemoBench extends MockLink {
    constructor() { super(); this.name = 'Demo (simulated servos)'; this.plugAt = Date.now() + 1200; this.count = 0; this.model = null; }
    /** null: STS3215s at mixed bauds · 7946: factory HD-1910s (the Microduck preset) */
    setModel(m) { this.model = m; }
    tick() {
      super.tick();
      const t = Date.now(), s = this.servos[0];
      if (!s && t > this.plugAt) {
        const bauds = [1000000, 1000000, 1000000, 115200, 500000];
        if (this.model === 7946) this.add(new MockServo(1, { model: 7946, maxVolt: 100, volt: 46, mode: 4, unload: 4, pos: 100 + Math.random() * 3800 }));
        else this.add(new MockServo(1, { baud: bauds[this.count % bauds.length], maxVolt: (this.count + 1) % 3 ? 140 : 80, volt: (this.count + 1) % 3 ? 121 : 73, pos: 1200 + Math.random() * 1600 }));
        this.count++;
      } else if (s && s.written && !s.unplugAt) s.unplugAt = t + 1800;   // numbered → the user unplugs it
      else if (s && s.unplugAt && t > s.unplugAt) { this.servos.length = 0; this.plugAt = t + 1500; }
    }
  }

  const api = { MockServo, MockLink, DemoBench };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.ArmDojoServoMock = api;
})(typeof self !== 'undefined' ? self : this);

window.ArmDojoServoI18n={"bg":{"start_here":"Свържете платката URT-2, за да започнете","start_here_det":"Включете URT-2 към този компютър с USB кабела и натиснете „Свържи URT-2“.","unsupported":"Този браузър не може да комуникира с USB устройства. Отворете страницата в Google Chrome или Microsoft Edge на компютър с Windows, Mac, Linux или ChromeOS.","unsupported_short":"Използвайте Chrome или Edge на компютър","step_connect":"Свържете URT-2","demo_badge":"Демо: симулирани сервомотори","btn_connect":"Свържи URT-2","btn_disconnect":"Прекъсни връзката","connect_hint":"Chrome показва намерените USB платки за сервомотори. Изберете URT-2: „USB Single Serial“ на Mac, „USB Serial Device (COM…)“ на Windows.","show_all_ports":"Моят адаптер не е в списъка","connected":"Свързано: {name}","connect_failed":"Портът не може да се отвори","disconnected":"Връзката е прекъсната","ready":"Свързано. Изберете ID и натиснете „Започни номерирането“.","ready_det":"Сервомоторите могат да се номерират само със захранване от USB. За центриране е нужно захранването на сервомоторите.","step_number":"Задайте ID на всеки сервомотор","start_id":"Начално ID","last_id":"Последно ID","presets":"Готови диапазони:","force_1m":"Задаване и на 1 Mbps за всички сервомотори (препоръчително)","speak":"Звуков сигнал и изговаряне на номера","one_at_a_time":"Включвайте САМО ЕДИН сервомотор наведнъж. Всички нови сервомотори идват с ID 1, затова два от тях на шината биха отговорили едновременно.","btn_start":"Започни номерирането","btn_stop":"Спри","plug_next":"Включете един сервомотор — той ще получи ID {n}","plug_next_det":"Скоростта на връзката се открива автоматично, каквато и да е.","remove_first":"Първо извадете сервомотор {id}, после включете следващия","remove_first_det":"Следващият сервомотор ще получи ID {n}.","already":"Този сервомотор вече има ID {id}. Включете следващия.","writing":"Записване на ID {n}…","done":"ID {n} е готово — извадете сервомотора и му сложете етикет","was_id":"беше ID {old}","new_model":"нов модел: изключете и включете захранването, след това проверете веригата веднъж","finished":"Готово до ID {end}","finished_det":"Свържете сервомоторите във верига и натиснете „Провери веригата“.","stopped":"Спряно","bad_range":"Началното ID трябва да е между 0 и последното ID (макс. 253)","col_new":"Ново ID","col_was":"Беше","col_model":"Модел","col_supply":"Захранване","supply_v":"захранване {v} V","io_lost":"Връзката с URT-2 е изгубена","io_lost_det":"Включете го отново и натиснете „Свържи URT-2“.","collision":"Изглежда, че е свързан повече от един сервомотор","collision_det":"Оставете включен само един сервомотор.","error":"Нещо се обърка: {msg}","error_det":"Проверете кабела; опитът ще се повтори автоматично.","step_check":"Проверете цялата верига","check_hint":"Свържете всички номерирани сервомотори във верига и сканирайте ID 0–253 при 1 Mbps.","btn_check":"Провери веригата","scanning":"Сканиране на ID 0–253…","found":"Намерени сервомотори: {n} (ID {ids})","none_found":"Нито един сервомотор не отговори при 1 Mbps. Проверете кабела или първо номерирайте сервомоторите.","duplicates":"Два или повече сервомотора имат едно и също ID {ids}. Извадете ги и ги номерирайте отново един по един.","missing":"Не са намерени: ID {ids}","chain_ok":"Отговарят всички сервомотори ({n}), без повтарящи се ID","chain_check_list":"Проверете списъка по-долу","step_center":"Центрирайте сервомотор (по желание)","center_hint":"Преди да монтирате накрайник или рамо, завъртете сервомотора в средно положение (2048). Свържете един сервомотор и захранването му. Валът ще се завърти.","btn_center":"Центрирай сервомотора","btn_release":"Освободи","center_search":"Търсене на сервомотора…","center_none":"Не е намерен точно един сервомотор. Свържете само един сервомотор.","center_unsupported":"Центрирането не е налично за {model}","centering":"Сервомотор {id} се завърта към средата…","centered":"Сервомотор {id} е центриран и държи позицията. Монтирайте накрайника и натиснете „Освободи“.","position":"позиция {p}","center_power":"Сервомоторът получава само {v} V","center_power_det":"Свържете захранването на сервомотора (7,4 V или 12 V според сервомотора).","released":"Освободен","log":"Технически лог (за поддръжката)","preset_duck":"Microduck","duck_note":"ID на ставите на Microduck, както ги използват репликите с HD-1910. Сервомоторите се номерират в този ред. Готовият диапазон отмята и защитата: защитата от свръхток и пренапрежение е изключена фабрично. Когато всички са готови, изключете и включете сервомоторите и натиснете „Провери веригата“, за да прочетете обратно ID и защитата.","j10":"дясно бедро, завъртане","j11":"дясно бедро, встрани","j12":"дясно бедро, напред/назад","j13":"дясно коляно","j14":"десен глезен","j20":"ляво бедро, завъртане","j21":"ляво бедро, встрани","j22":"ляво бедро, напред/назад","j23":"ляво коляно","j24":"ляв глезен","j30":"врат, горе/долу","j31":"глава, горе/долу","j32":"глава, завъртане","j33":"глава, наклон","j34":"клюн","protect":"Включване на защита от свръхток и пренапрежение (фабрично изключена; препоръчително за HD-1910)","protect_done":"защитата от свръхток и пренапрежение е включена","protect_skipped":"защитата не е променена: не е налична за този модел","col_prot":"Защита","prot_cell":"ток {oc} · напрежение {ov}","joint_is":"ID {n} = {joint}"},"cs":{"start_here":"Pro začátek připojte desku URT-2","start_here_det":"Zapojte URT-2 do tohoto počítače kabelem USB a pak klikněte na „Připojit URT-2“.","unsupported":"Tento prohlížeč neumí komunikovat se zařízeními USB. Otevřete tuto stránku v prohlížeči Google Chrome nebo Microsoft Edge na počítači se systémem Windows, Linux nebo ChromeOS nebo na počítači Mac.","unsupported_short":"Použijte Chrome nebo Edge na počítači","step_connect":"Připojte URT-2","demo_badge":"Demo: simulovaná serva","btn_connect":"Připojit URT-2","btn_disconnect":"Odpojit","connect_hint":"Chrome zobrazí nalezené USB desky pro serva. Vyberte URT-2: na Macu „USB Single Serial“, ve Windows „Sériové zařízení USB (COM…)“.","show_all_ports":"Můj adaptér v seznamu není","connected":"Připojeno: {name}","connect_failed":"Port se nepodařilo otevřít","disconnected":"Odpojeno","ready":"Připojeno. Zvolte ID a klikněte na „Spustit číslování“.","ready_det":"Serva můžete číslovat jen s napájením z USB. K vycentrování potřebujete napájecí zdroj pro serva.","step_number":"Přidělte každému servu jeho ID","start_id":"Začít od ID","last_id":"Poslední ID","presets":"Předvolby:","force_1m":"Nastavit také všechna serva na 1 Mbps (doporučeno)","speak":"Pípnout a vyslovit číslo","one_at_a_time":"Zapojujte vždy JEN JEDNO servo. Všechna nová serva mají ID 1, takže dvě serva na sběrnici by odpověděla současně.","btn_start":"Spustit číslování","btn_stop":"Zastavit","plug_next":"Zapojte jedno servo – dostane ID {n}","plug_next_det":"Přenosovou rychlost nástroj zjistí sám, ať je jakákoli.","remove_first":"Nejdřív odpojte servo {id}, pak zapojte další","remove_first_det":"Další servo dostane ID {n}.","already":"Toto servo už dostalo ID {id}. Zapojte další.","writing":"Zapisování ID {n}…","done":"ID {n} hotovo – servo odpojte a označte","was_id":"dříve ID {old}","new_model":"nový model: vypněte a zapněte napájení a pak jednou zkontrolujte řetěz","finished":"Hotovo až po ID {end}","finished_det":"Zapojte serva do řetězu a klikněte na „Zkontrolovat řetěz“.","stopped":"Zastaveno","bad_range":"Počáteční ID musí být mezi 0 a posledním ID (max. 253)","col_new":"Nové ID","col_was":"Dříve","col_model":"Model","col_supply":"Napájení","supply_v":"napájení {v} V","io_lost":"Spojení s URT-2 se přerušilo","io_lost_det":"Znovu ji zapojte a klikněte na „Připojit URT-2“.","collision":"Zdá se, že je připojeno více než jedno servo","collision_det":"Nechte zapojené jen jedno servo.","error":"Něco se pokazilo: {msg}","error_det":"Zkontrolujte kabel; nástroj to zkusí znovu.","step_check":"Zkontrolujte celý řetěz","check_hint":"Zapojte všechna očíslovaná serva do řetězu a prohledejte ID 0–253 při 1 Mbps.","btn_check":"Zkontrolovat řetěz","scanning":"Prohledávání ID 0–253…","found":"Nalezená serva: {n} (ID {ids})","none_found":"Při 1 Mbps neodpovědělo žádné servo. Zkontrolujte kabel nebo serva nejdřív očíslujte.","duplicates":"Dvě nebo více serv mají stejné ID {ids}. Odpojte je a očíslujte je znovu, vždy po jednom.","missing":"Nenalezeno: ID {ids}","chain_ok":"Všechna serva odpovídají ({n}), žádné duplicity","chain_check_list":"Zkontrolujte seznam níže","step_center":"Vycentrujte servo (volitelné)","center_hint":"Než nasadíte páku serva nebo rameno, otočte servo do středové polohy (2048). Připojte jedno servo a jeho napájecí zdroj. Hřídel se otočí.","btn_center":"Vycentrovat servo","btn_release":"Uvolnit","center_search":"Hledání serva…","center_none":"Nepodařilo se najít právě jedno servo. Připojte přesně jedno servo.","center_unsupported":"Vycentrování není pro {model} k dispozici","centering":"Otáčení serva {id} do středu…","centered":"Servo {id} je ve středu a drží polohu. Nasaďte páku a pak klikněte na „Uvolnit“.","position":"poloha {p}","center_power":"Servo dostává jen {v} V","center_power_det":"Připojte napájecí zdroj serva (7,4 V nebo 12 V podle toho, co vaše servo potřebuje).","released":"Uvolněno","log":"Technický protokol (pro podporu)","preset_duck":"Microduck","duck_note":"ID kloubů Microducku, jak je používají repliky s HD-1910. Serva se číslují v tomto pořadí. Předvolba také zaškrtne ochranu: nadproudová a přepěťová ochrana je z výroby vypnutá. Až budou hotová všechna, vypněte a zapněte serva a klikněte na „Zkontrolovat řetěz“, aby se ID a ochrana načetly zpět.","j10":"pravá kyčel, otočení","j11":"pravá kyčel, do strany","j12":"pravá kyčel, vpřed/vzad","j13":"pravé koleno","j14":"pravý kotník","j20":"levá kyčel, otočení","j21":"levá kyčel, do strany","j22":"levá kyčel, vpřed/vzad","j23":"levé koleno","j24":"levý kotník","j30":"krk, nahoru/dolů","j31":"hlava, nahoru/dolů","j32":"hlava, otočení","j33":"hlava, náklon","j34":"zobák","protect":"Zapnout nadproudovou a přepěťovou ochranu (z výroby vypnutá; doporučeno pro HD-1910)","protect_done":"nadproudová a přepěťová ochrana zapnuta","protect_skipped":"ochrana nezměněna: u tohoto modelu není k dispozici","col_prot":"Ochrana","prot_cell":"proud {oc} · napětí {ov}","joint_is":"ID {n} = {joint}"},"da":{"start_here":"Tilslut dit URT-2-kort for at starte","start_here_det":"Sæt URT-2 i denne computer med USB-kablet, og klik derefter på »Tilslut URT-2«.","unsupported":"Denne browser kan ikke kommunikere med USB-enheder. Åbn siden i Google Chrome eller Microsoft Edge på en computer med Windows, Mac, Linux eller ChromeOS.","unsupported_short":"Brug Chrome eller Edge på en computer","step_connect":"Tilslut URT-2","demo_badge":"Demo: simulerede servoer","btn_connect":"Tilslut URT-2","btn_disconnect":"Afbryd","connect_hint":"Chrome viser de USB-servokort, den finder. Vælg URT-2: »USB Single Serial« på en Mac, »USB Serial Device (COM…)« på Windows.","show_all_ports":"Min adapter er ikke på listen","connected":"Tilsluttet: {name}","connect_failed":"Porten kunne ikke åbnes","disconnected":"Afbrudt","ready":"Tilsluttet. Vælg ID'erne, og klik på »Start nummerering«.","ready_det":"Du kan nummerere servoer med kun strøm fra USB. Centrering kræver servoernes strømforsyning.","step_number":"Giv hver servo sit ID","start_id":"Start ved ID","last_id":"Sidste ID","presets":"Forvalg:","force_1m":"Indstil også alle servoer til 1 Mbps (anbefales)","speak":"Bip og sig nummeret","one_at_a_time":"Sæt ÉN servo i ad gangen. Alle nye servoer har ID 1, så to af dem på bussen ville svare samtidig.","btn_start":"Start nummerering","btn_stop":"Stop","plug_next":"Sæt én servo i — den får ID {n}","plug_next_det":"Baudraten findes automatisk, uanset hvad den er.","remove_first":"Tag først servo {id} ud, og sæt derefter den næste i","remove_first_det":"Den næste servo får ID {n}.","already":"Denne servo har allerede fået ID {id}. Sæt den næste i.","writing":"Skriver ID {n}…","done":"ID {n} er klar — tag servoen ud, og mærk den","was_id":"var ID {old}","new_model":"ny model: sluk og tænd for strømmen, og tjek derefter kæden én gang","finished":"Færdig til og med ID {end}","finished_det":"Forbind servoerne i en kæde, og klik på »Tjek kæden«.","stopped":"Stoppet","bad_range":"Start-ID skal være mellem 0 og sidste ID (maks. 253)","col_new":"Nyt ID","col_was":"Før","col_model":"Model","col_supply":"Forsyning","supply_v":"Forsyning {v} V","io_lost":"Forbindelsen til URT-2 blev afbrudt","io_lost_det":"Sæt kortet i igen, og klik på »Tilslut URT-2«.","collision":"Der ser ud til at være mere end én servo tilsluttet","collision_det":"Hav kun én servo sat i.","error":"Noget gik galt: {msg}","error_det":"Tjek kablet; værktøjet prøver igen.","step_check":"Tjek hele kæden","check_hint":"Forbind alle nummererede servoer i en kæde, og scan ID 0–253 ved 1 Mbps.","btn_check":"Tjek kæden","scanning":"Scanner ID 0–253…","found":"Fundne servoer: {n} (ID {ids})","none_found":"Ingen servo svarede ved 1 Mbps. Tjek kablet, eller nummerér servoerne først.","duplicates":"To eller flere servoer har samme ID {ids}. Tag dem ud, og nummerér dem igen én ad gangen.","missing":"Ikke fundet: ID {ids}","chain_ok":"Alle servoer svarer ({n}), ingen dubletter","chain_check_list":"Tjek listen nedenfor","step_center":"Centrér en servo (valgfrit)","center_hint":"Før du monterer et servohorn eller en arm, skal du dreje servoen til midten (2048). Tilslut én servo og dens strømforsyning. Akslen drejer.","btn_center":"Centrér servo","btn_release":"Slip","center_search":"Leder efter servoen…","center_none":"Der blev ikke fundet én enkelt servo. Tilslut præcis én servo.","center_unsupported":"Centrering er ikke tilgængelig for {model}","centering":"Drejer servo {id} til midten…","centered":"Servo {id} er centreret og holder positionen. Montér servohornet, og klik derefter på »Slip«.","position":"position {p}","center_power":"Servoen får kun {v} V","center_power_det":"Tilslut servoens strømforsyning (7,4 V eller 12 V, alt efter hvad din servo skal bruge).","released":"Sluppet","log":"Teknisk log (til support)","preset_duck":"Microduck","duck_note":"Microduck-led-ID'er, som HD-1910-replikaerne bruger dem. Servoerne nummereres i denne rækkefølge. Forvalget sætter også flueben ved beskyttelsen: overstrøms- og overspændingsbeskyttelse er slået fra fra fabrikken. Når alle er færdige, så sluk og tænd servoerne, og klik på »Tjek kæden« for at læse ID'er og beskyttelse tilbage.","j10":"højre hofte, drej","j11":"højre hofte, sideværts","j12":"højre hofte, frem/tilbage","j13":"højre knæ","j14":"højre ankel","j20":"venstre hofte, drej","j21":"venstre hofte, sideværts","j22":"venstre hofte, frem/tilbage","j23":"venstre knæ","j24":"venstre ankel","j30":"hals, op/ned","j31":"hoved, op/ned","j32":"hoved, drej","j33":"hoved, vip","j34":"næb","protect":"Slå overstrøms- og overspændingsbeskyttelse til (slået fra fra fabrikken; anbefales til HD-1910)","protect_done":"overstrøms- og overspændingsbeskyttelse slået til","protect_skipped":"beskyttelse ikke ændret: findes ikke på denne model","col_prot":"Beskyttelse","prot_cell":"strøm {oc} · spænding {ov}","joint_is":"ID {n} = {joint}"},"de":{"start_here":"Verbinden Sie Ihr URT-2-Board, um zu starten","start_here_det":"Stecken Sie das URT-2 mit dem USB-Kabel an diesen Computer und klicken Sie dann auf „URT-2 verbinden“.","unsupported":"Dieser Browser kann nicht mit USB-Geräten kommunizieren. Öffnen Sie diese Seite in Google Chrome oder Microsoft Edge auf einem Windows-, Mac-, Linux- oder ChromeOS-Computer.","unsupported_short":"Bitte nutzen Sie Chrome oder Edge auf einem Computer","step_connect":"URT-2 verbinden","demo_badge":"Demo: simulierte Servos","btn_connect":"URT-2 verbinden","btn_disconnect":"Trennen","connect_hint":"Chrome zeigt die gefundenen USB-Servo-Boards an. Wählen Sie das URT-2: „USB Single Serial“ auf dem Mac, „Serielles USB-Gerät (COM…)“ unter Windows.","show_all_ports":"Mein Adapter ist nicht in der Liste","connected":"Verbunden: {name}","connect_failed":"Der Port konnte nicht geöffnet werden","disconnected":"Getrennt","ready":"Verbunden. Wählen Sie die IDs und klicken Sie auf „Nummerierung starten“.","ready_det":"Zum Nummerieren reicht der Strom über USB. Zum Zentrieren brauchen Sie das Servo-Netzteil.","step_number":"Jedem Servo seine ID geben","start_id":"Start-ID","last_id":"Letzte ID","presets":"Vorlagen:","force_1m":"Alle Servos zusätzlich auf 1 Mbps stellen (empfohlen)","speak":"Piepton und Nummer ansagen","one_at_a_time":"Schließen Sie immer nur EIN Servo an. Alle neuen Servos haben ab Werk die ID 1 – zwei davon am Bus würden gleichzeitig antworten.","btn_start":"Nummerierung starten","btn_stop":"Stopp","plug_next":"Schließen Sie ein Servo an – es bekommt die ID {n}","plug_next_det":"Die Baudrate wird automatisch erkannt.","remove_first":"Ziehen Sie zuerst Servo {id} ab und schließen Sie dann das nächste an","remove_first_det":"Das nächste Servo bekommt die ID {n}.","already":"Dieses Servo hat bereits die ID {id}. Schließen Sie das nächste an.","writing":"ID {n} wird geschrieben…","done":"ID {n} fertig – Servo abziehen und beschriften","was_id":"vorher ID {old}","new_model":"neues Modell: einmal aus- und wieder einschalten, dann die Kette einmal prüfen","finished":"Alles fertig bis ID {end}","finished_det":"Verbinden Sie die Servos zu einer Kette und klicken Sie auf „Kette prüfen“.","stopped":"Gestoppt","bad_range":"Die Start-ID muss zwischen 0 und der letzten ID liegen (max. 253)","col_new":"Neue ID","col_was":"Vorher","col_model":"Modell","col_supply":"Versorgung","supply_v":"Versorgung {v} V","io_lost":"Verbindung zum URT-2 verloren","io_lost_det":"Stecken Sie es wieder ein und klicken Sie auf „URT-2 verbinden“.","collision":"Anscheinend ist mehr als ein Servo angeschlossen","collision_det":"Lassen Sie nur ein Servo angeschlossen.","error":"Etwas ist schiefgelaufen: {msg}","error_det":"Prüfen Sie das Kabel – es wird automatisch erneut versucht.","step_check":"Die ganze Kette prüfen","check_hint":"Verbinden Sie alle nummerierten Servos zu einer Kette und scannen Sie die IDs 0–253 bei 1 Mbps.","btn_check":"Kette prüfen","scanning":"IDs 0–253 werden gescannt…","found":"Gefundene Servos: {n} (ID {ids})","none_found":"Bei 1 Mbps hat kein Servo geantwortet. Prüfen Sie das Kabel oder nummerieren Sie die Servos zuerst.","duplicates":"Zwei oder mehr Servos haben die ID {ids}. Ziehen Sie sie ab und nummerieren Sie sie einzeln neu.","missing":"Nicht gefunden: ID {ids}","chain_ok":"Alle Servos antworten ({n}), keine doppelten IDs","chain_check_list":"Prüfen Sie die Liste unten","step_center":"Servo zentrieren (optional)","center_hint":"Bevor Sie ein Servohorn oder einen Arm montieren, drehen Sie das Servo in die Mitte (2048). Schließen Sie ein Servo und sein Netzteil an. Die Welle dreht sich.","btn_center":"Servo zentrieren","btn_release":"Freigeben","center_search":"Servo wird gesucht…","center_none":"Kein einzelnes Servo gefunden. Schließen Sie genau ein Servo an.","center_unsupported":"Zentrieren ist für {model} nicht verfügbar","centering":"Servo {id} wird in die Mitte gedreht…","centered":"Servo {id} ist zentriert und hält die Position. Montieren Sie das Horn und klicken Sie dann auf „Freigeben“.","position":"Position {p}","center_power":"Das Servo bekommt nur {v} V","center_power_det":"Schließen Sie das Servo-Netzteil an (7,4 V oder 12 V, je nach Servo).","released":"Freigegeben","log":"Technisches Protokoll (für den Support)","preset_duck":"Microduck","duck_note":"Microduck-Gelenk-IDs, wie sie die HD-1910-Nachbauten verwenden. Die Servos werden in dieser Reihenfolge nummeriert. Die Vorlage setzt auch das Häkchen beim Schutz: Überstrom- und Überspannungsschutz sind ab Werk aus. Wenn alle fertig sind, schalten Sie die Servos aus und wieder ein und klicken Sie auf „Kette prüfen“, um IDs und Schutz zurückzulesen.","j10":"rechte Hüfte, Drehung","j11":"rechte Hüfte, seitlich","j12":"rechte Hüfte, vor/zurück","j13":"rechtes Knie","j14":"rechter Knöchel","j20":"linke Hüfte, Drehung","j21":"linke Hüfte, seitlich","j22":"linke Hüfte, vor/zurück","j23":"linkes Knie","j24":"linker Knöchel","j30":"Hals, hoch/runter","j31":"Kopf, hoch/runter","j32":"Kopf, Drehung","j33":"Kopf, Neigung","j34":"Schnabel","protect":"Überstrom- und Überspannungsschutz einschalten (ab Werk aus; empfohlen für HD-1910)","protect_done":"Überstrom- und Überspannungsschutz an","protect_skipped":"Schutz nicht geändert: bei diesem Modell nicht verfügbar","col_prot":"Schutz","prot_cell":"Strom {oc} · Spannung {ov}","joint_is":"ID {n} = {joint}"},"el":{"start_here":"Συνδέστε την πλακέτα URT-2 για να ξεκινήσετε","start_here_det":"Συνδέστε το URT-2 σε αυτόν τον υπολογιστή με το καλώδιο USB και πατήστε «Σύνδεση URT-2».","unsupported":"Αυτό το πρόγραμμα περιήγησης δεν μπορεί να επικοινωνήσει με συσκευές USB. Ανοίξτε τη σελίδα στο Google Chrome ή στο Microsoft Edge σε υπολογιστή με Windows, Mac, Linux ή ChromeOS.","unsupported_short":"Χρησιμοποιήστε Chrome ή Edge σε υπολογιστή","step_connect":"Σύνδεση του URT-2","demo_badge":"Επίδειξη: προσομοιωμένοι σερβοκινητήρες","btn_connect":"Σύνδεση URT-2","btn_disconnect":"Αποσύνδεση","connect_hint":"Το Chrome εμφανίζει τις πλακέτες σερβοκινητήρων USB που βρίσκει. Επιλέξτε το URT-2: «USB Single Serial» σε Mac, «USB Serial Device (COM…)» σε Windows.","show_all_ports":"Ο προσαρμογέας μου δεν είναι στη λίστα","connected":"Συνδέθηκε: {name}","connect_failed":"Δεν ήταν δυνατό να ανοίξει η θύρα","disconnected":"Αποσυνδέθηκε","ready":"Συνδέθηκε. Επιλέξτε τα ID και πατήστε «Έναρξη αρίθμησης».","ready_det":"Μπορείτε να αριθμήσετε σερβοκινητήρες μόνο με την τροφοδοσία του USB. Για το κεντράρισμα χρειάζεται το τροφοδοτικό των σερβοκινητήρων.","step_number":"Δώστε σε κάθε σερβοκινητήρα το ID του","start_id":"Αρχικό ID","last_id":"Τελευταίο ID","presets":"Έτοιμες επιλογές:","force_1m":"Ρύθμιση όλων των σερβοκινητήρων και σε 1 Mbps (συνιστάται)","speak":"Ήχος και εκφώνηση του αριθμού","one_at_a_time":"Συνδέετε ΕΝΑΝ σερβοκινητήρα κάθε φορά. Όλοι οι καινούργιοι σερβοκινητήρες έρχονται με ID 1, οπότε δύο στον δίαυλο θα απαντούσαν μαζί.","btn_start":"Έναρξη αρίθμησης","btn_stop":"Διακοπή","plug_next":"Συνδέστε έναν σερβοκινητήρα — θα πάρει το ID {n}","plug_next_det":"Η ταχύτητα επικοινωνίας εντοπίζεται αυτόματα, όποια κι αν είναι.","remove_first":"Αποσυνδέστε πρώτα τον σερβοκινητήρα {id} και μετά συνδέστε τον επόμενο","remove_first_det":"Ο επόμενος σερβοκινητήρας θα πάρει το ID {n}.","already":"Αυτός ο σερβοκινητήρας έχει ήδη το ID {id}. Συνδέστε τον επόμενο.","writing":"Εγγραφή ID {n}…","done":"Το ID {n} ολοκληρώθηκε — αποσυνδέστε τον σερβοκινητήρα και βάλτε του ετικέτα","was_id":"πριν: ID {old}","new_model":"νέο μοντέλο: κλείστε και ανοίξτε την τροφοδοσία και ελέγξτε μία φορά την αλυσίδα","finished":"Όλα έτοιμα έως το ID {end}","finished_det":"Συνδέστε τους σερβοκινητήρες σε αλυσίδα και πατήστε «Έλεγχος αλυσίδας».","stopped":"Διακόπηκε","bad_range":"Το αρχικό ID πρέπει να είναι μεταξύ 0 και του τελευταίου ID (έως 253)","col_new":"Νέο ID","col_was":"Πριν","col_model":"Μοντέλο","col_supply":"Τροφοδοσία","supply_v":"τροφοδοσία {v} V","io_lost":"Χάθηκε η σύνδεση με το URT-2","io_lost_det":"Συνδέστε το ξανά και πατήστε «Σύνδεση URT-2».","collision":"Φαίνεται ότι είναι συνδεδεμένοι περισσότεροι από ένας σερβοκινητήρες","collision_det":"Αφήστε συνδεδεμένο μόνο έναν σερβοκινητήρα.","error":"Κάτι πήγε στραβά: {msg}","error_det":"Ελέγξτε το καλώδιο· θα γίνει αυτόματα νέα προσπάθεια.","step_check":"Έλεγχος όλης της αλυσίδας","check_hint":"Συνδέστε όλους τους αριθμημένους σερβοκινητήρες σε αλυσίδα και σαρώστε τα ID 0–253 στο 1 Mbps.","btn_check":"Έλεγχος αλυσίδας","scanning":"Σάρωση ID 0–253…","found":"Βρέθηκαν σερβοκινητήρες: {n} (ID {ids})","none_found":"Κανένας σερβοκινητήρας δεν απάντησε στο 1 Mbps. Ελέγξτε το καλώδιο ή αριθμήστε πρώτα τους σερβοκινητήρες.","duplicates":"Δύο ή περισσότεροι σερβοκινητήρες έχουν το ίδιο ID {ids}. Αποσυνδέστε τους και αριθμήστε τους ξανά έναν έναν.","missing":"Δεν βρέθηκαν: ID {ids}","chain_ok":"Απαντούν όλοι οι σερβοκινητήρες ({n}), χωρίς διπλά ID","chain_check_list":"Ελέγξτε τη λίστα παρακάτω","step_center":"Κεντράρισμα σερβοκινητήρα (προαιρετικό)","center_hint":"Πριν τοποθετήσετε δίσκο ή βραχίονα στον άξονα, γυρίστε τον σερβοκινητήρα στη μεσαία θέση (2048). Συνδέστε έναν σερβοκινητήρα και το τροφοδοτικό του. Ο άξονας θα περιστραφεί.","btn_center":"Κεντράρισμα σερβοκινητήρα","btn_release":"Απελευθέρωση","center_search":"Αναζήτηση σερβοκινητήρα…","center_none":"Δεν βρέθηκε ένας μόνο σερβοκινητήρας. Συνδέστε ακριβώς έναν σερβοκινητήρα.","center_unsupported":"Το κεντράρισμα δεν είναι διαθέσιμο για το {model}","centering":"Ο σερβοκινητήρας {id} γυρίζει στη μέση…","centered":"Ο σερβοκινητήρας {id} είναι κεντραρισμένος και κρατά τη θέση του. Τοποθετήστε τον δίσκο και πατήστε «Απελευθέρωση».","position":"θέση {p}","center_power":"Ο σερβοκινητήρας παίρνει μόνο {v} V","center_power_det":"Συνδέστε το τροφοδοτικό του σερβοκινητήρα (7,4 V ή 12 V, ανάλογα με τον σερβοκινητήρα).","released":"Απελευθερώθηκε","log":"Τεχνικό αρχείο καταγραφής (για υποστήριξη)","preset_duck":"Microduck","duck_note":"Τα ID των αρθρώσεων του Microduck, όπως τα χρησιμοποιούν τα αντίγραφα με HD-1910. Οι σερβοκινητήρες αριθμούνται με αυτή τη σειρά. Η έτοιμη επιλογή τσεκάρει και την προστασία: η προστασία από υπερένταση και υπέρταση είναι εργοστασιακά απενεργοποιημένη. Όταν τελειώσουν όλοι, σβήστε και ανάψτε ξανά τους σερβοκινητήρες και πατήστε «Έλεγχος αλυσίδας» για να διαβαστούν ξανά τα ID και η προστασία.","j10":"δεξί ισχίο, περιστροφή","j11":"δεξί ισχίο, πλάγια","j12":"δεξί ισχίο, μπρος/πίσω","j13":"δεξί γόνατο","j14":"δεξιός αστράγαλος","j20":"αριστερό ισχίο, περιστροφή","j21":"αριστερό ισχίο, πλάγια","j22":"αριστερό ισχίο, μπρος/πίσω","j23":"αριστερό γόνατο","j24":"αριστερός αστράγαλος","j30":"λαιμός, πάνω/κάτω","j31":"κεφάλι, πάνω/κάτω","j32":"κεφάλι, περιστροφή","j33":"κεφάλι, κλίση","j34":"ράμφος","protect":"Ενεργοποίηση προστασίας από υπερένταση και υπέρταση (εργοστασιακά ανενεργή· συνιστάται για HD-1910)","protect_done":"προστασία από υπερένταση και υπέρταση ενεργή","protect_skipped":"η προστασία δεν άλλαξε: δεν υπάρχει σε αυτό το μοντέλο","col_prot":"Προστασία","prot_cell":"ρεύμα {oc} · τάση {ov}","joint_is":"ID {n} = {joint}"},"en":{"start_here":"Connect your URT-2 board to start","start_here_det":"Plug the URT-2 into this computer with its USB cable, then press “Connect URT-2”.","unsupported":"This browser cannot talk to USB devices. Open this page in Google Chrome or Microsoft Edge on a Windows, Mac, Linux or ChromeOS computer.","unsupported_short":"Please use Chrome or Edge on a computer","step_connect":"Connect the URT-2","demo_badge":"Demo: simulated servos","btn_connect":"Connect URT-2","btn_disconnect":"Disconnect","connect_hint":"Chrome lists the USB servo boards it finds. Pick the URT-2: “USB Single Serial” on a Mac, “USB Serial Device (COM…)” on Windows.","show_all_ports":"My adapter is not in the list","connected":"Connected: {name}","connect_failed":"Could not open the port","disconnected":"Disconnected","ready":"Connected. Choose the IDs and press “Start numbering”.","ready_det":"You can number servos with only USB power. Centering needs the servo power supply.","step_number":"Give each servo its ID","start_id":"Start at ID","last_id":"Last ID","presets":"Presets:","force_1m":"Also set every servo to 1 Mbps (recommended)","speak":"Beep and say the number","one_at_a_time":"Plug in ONE servo at a time. All new servos come as ID 1, so two of them on the bus would answer together.","btn_start":"Start numbering","btn_stop":"Stop","plug_next":"Plug in one servo — it becomes ID {n}","plug_next_det":"Any baud rate is found automatically.","remove_first":"Unplug servo {id} first, then plug in the next one","remove_first_det":"The next servo becomes ID {n}.","already":"This servo already got ID {id}. Plug in the next one.","writing":"Writing ID {n}…","done":"ID {n} done — unplug it and label it","was_id":"was ID {old}","new_model":"new model: power it off and on, then check the chain once","finished":"All done up to ID {end}","finished_det":"Chain the servos together and press “Check the chain”.","stopped":"Stopped","bad_range":"Start ID must be between 0 and the last ID (max 253)","col_new":"New ID","col_was":"Was","col_model":"Model","col_supply":"Supply","supply_v":"{v} V supply","io_lost":"Lost the connection to the URT-2","io_lost_det":"Plug it back in and press “Connect URT-2”.","collision":"More than one servo seems to be connected","collision_det":"Keep only one servo plugged in.","error":"Something went wrong: {msg}","error_det":"Check the cable; it will retry.","step_check":"Check the whole chain","check_hint":"Connect all numbered servos in a chain and scan IDs 0–253 at 1 Mbps.","btn_check":"Check the chain","scanning":"Scanning IDs 0–253…","found":"Servos found: {n} (ID {ids})","none_found":"No servo answered at 1 Mbps. Check the cable, or number the servos first.","duplicates":"Two or more servos share ID {ids}. Unplug them and number them again one at a time.","missing":"Not found: ID {ids}","chain_ok":"All servos answer ({n}), no duplicates","chain_check_list":"Check the list below","step_center":"Center a servo (optional)","center_hint":"Before you fit a horn or an arm, turn the servo to its middle (2048). Connect one servo and its power supply. The shaft will turn.","btn_center":"Center servo","btn_release":"Release","center_search":"Looking for the servo…","center_none":"No single servo found. Connect exactly one servo.","center_unsupported":"Centering is not available for {model}","centering":"Turning servo {id} to the middle…","centered":"Servo {id} is centered and holding. Fit the horn, then press “Release”.","position":"position {p}","center_power":"The servo only gets {v} V","center_power_det":"Connect the servo power supply (7.4 V or 12 V, as your servo needs).","released":"Released","log":"Technical log (for support)","preset_duck":"Microduck","duck_note":"Microduck joint IDs, as used by the HD-1910 replica builds. The servos are numbered in this order. The preset also ticks the protection box: over-current and over-voltage protection are off from the factory. When all are done, power the servos off and on, then press “Check the chain” to read the IDs and the protection back.","j10":"right hip yaw","j11":"right hip roll","j12":"right hip pitch","j13":"right knee","j14":"right ankle","j20":"left hip yaw","j21":"left hip roll","j22":"left hip pitch","j23":"left knee","j24":"left ankle","j30":"neck pitch","j31":"head pitch","j32":"head yaw","j33":"head roll","j34":"beak","protect":"Switch on over-current and over-voltage protection (off from the factory; recommended for HD-1910)","protect_done":"over-current and over-voltage protection on","protect_skipped":"protection not changed: not available on this model","col_prot":"Protection","prot_cell":"current {oc} · voltage {ov}","joint_is":"ID {n} = {joint}"},"es":{"start_here":"Conecte su placa URT-2 para empezar","start_here_det":"Conecte la URT-2 a este ordenador con su cable USB y pulse «Conectar URT-2».","unsupported":"Este navegador no puede comunicarse con dispositivos USB. Abra esta página en Google Chrome o Microsoft Edge en un ordenador con Windows, Mac, Linux o ChromeOS.","unsupported_short":"Use Chrome o Edge en un ordenador","step_connect":"Conectar la URT-2","demo_badge":"Demo: servos simulados","btn_connect":"Conectar URT-2","btn_disconnect":"Desconectar","connect_hint":"Chrome muestra las placas de servos USB que encuentra. Elija la URT-2: «USB Single Serial» en Mac, «USB Serial Device (COM…)» en Windows.","show_all_ports":"Mi adaptador no aparece en la lista","connected":"Conectado: {name}","connect_failed":"No se pudo abrir el puerto","disconnected":"Desconectado","ready":"Conectado. Elija los ID y pulse «Iniciar numeración».","ready_det":"Para numerar los servos basta con la alimentación USB. Para centrarlos hace falta la fuente de alimentación de los servos.","step_number":"Asignar un ID a cada servo","start_id":"ID inicial","last_id":"Último ID","presets":"Predefinidos:","force_1m":"Poner también todos los servos a 1 Mbps (recomendado)","speak":"Pitar y decir el número","one_at_a_time":"Conecte UN solo servo cada vez. Todos los servos nuevos vienen con el ID 1, así que dos en el bus responderían a la vez.","btn_start":"Iniciar numeración","btn_stop":"Detener","plug_next":"Conecte un servo: recibirá el ID {n}","plug_next_det":"La velocidad en baudios se detecta automáticamente.","remove_first":"Desconecte primero el servo {id} y luego conecte el siguiente","remove_first_det":"El siguiente servo recibirá el ID {n}.","already":"Este servo ya tiene el ID {id}. Conecte el siguiente.","writing":"Escribiendo el ID {n}…","done":"ID {n} listo: desconecte el servo y etiquételo","was_id":"antes ID {old}","new_model":"modelo nuevo: apáguelo y vuelva a encenderlo, y después compruebe la cadena una vez","finished":"Todo listo hasta el ID {end}","finished_det":"Conecte los servos en cadena y pulse «Comprobar la cadena».","stopped":"Detenido","bad_range":"El ID inicial debe estar entre 0 y el último ID (máx. 253)","col_new":"ID nuevo","col_was":"Antes","col_model":"Modelo","col_supply":"Alimentación","supply_v":"alimentación de {v} V","io_lost":"Se ha perdido la conexión con la URT-2","io_lost_det":"Vuelva a conectarla y pulse «Conectar URT-2».","collision":"Parece que hay más de un servo conectado","collision_det":"Deje un solo servo conectado.","error":"Algo ha fallado: {msg}","error_det":"Revise el cable; se volverá a intentar.","step_check":"Comprobar toda la cadena","check_hint":"Conecte en cadena todos los servos numerados y escanee los ID 0–253 a 1 Mbps.","btn_check":"Comprobar la cadena","scanning":"Escaneando los ID 0–253…","found":"Servos encontrados: {n} (ID {ids})","none_found":"Ningún servo ha respondido a 1 Mbps. Revise el cable o numere primero los servos.","duplicates":"Dos o más servos comparten el ID {ids}. Desconéctelos y vuelva a numerarlos de uno en uno.","missing":"No encontrados: ID {ids}","chain_ok":"Responden todos los servos ({n}), sin duplicados","chain_check_list":"Revise la lista de abajo","step_center":"Centrar un servo (opcional)","center_hint":"Antes de montar una cruceta o un brazo, lleve el servo a su posición central (2048). Conecte un servo y su fuente de alimentación. El eje girará.","btn_center":"Centrar servo","btn_release":"Soltar","center_search":"Buscando el servo…","center_none":"No se ha encontrado un único servo. Conecte exactamente un servo.","center_unsupported":"El centrado no está disponible para {model}","centering":"Girando el servo {id} a la posición central…","centered":"El servo {id} está centrado y mantiene la posición. Monte la cruceta y pulse «Soltar».","position":"posición {p}","center_power":"El servo solo recibe {v} V","center_power_det":"Conecte la fuente de alimentación de los servos (7,4 V o 12 V, según su servo).","released":"Soltado","log":"Registro técnico (para soporte)","preset_duck":"Microduck","duck_note":"ID de las articulaciones del Microduck, tal como las usan las réplicas con HD-1910. Los servos se numeran en este orden. El predefinido también marca la casilla de protección: la protección contra sobrecorriente y sobretensión viene desactivada de fábrica. Cuando termine con todos, apague y encienda los servos y pulse «Comprobar la cadena» para volver a leer los ID y la protección.","j10":"cadera derecha, giro","j11":"cadera derecha, lateral","j12":"cadera derecha, adelante/atrás","j13":"rodilla derecha","j14":"tobillo derecho","j20":"cadera izquierda, giro","j21":"cadera izquierda, lateral","j22":"cadera izquierda, adelante/atrás","j23":"rodilla izquierda","j24":"tobillo izquierdo","j30":"cuello, arriba/abajo","j31":"cabeza, arriba/abajo","j32":"cabeza, giro","j33":"cabeza, inclinación","j34":"pico","protect":"Activar la protección contra sobrecorriente y sobretensión (desactivada de fábrica; recomendado para HD-1910)","protect_done":"protección contra sobrecorriente y sobretensión activada","protect_skipped":"protección sin cambios: no disponible en este modelo","col_prot":"Protección","prot_cell":"corriente {oc} · tensión {ov}","joint_is":"ID {n} = {joint}"},"fi":{"start_here":"Aloita yhdistämällä URT-2-kortti","start_here_det":"Kytke URT-2 tähän tietokoneeseen USB-kaapelilla ja paina sitten ”Yhdistä URT-2”.","unsupported":"Tämä selain ei pysty käyttämään USB-laitteita. Avaa tämä sivu Google Chrome- tai Microsoft Edge -selaimessa Windows-, Mac-, Linux- tai ChromeOS-tietokoneella.","unsupported_short":"Käytä tietokoneella Chrome- tai Edge-selainta","step_connect":"Yhdistä URT-2","demo_badge":"Demo: simuloidut servot","btn_connect":"Yhdistä URT-2","btn_disconnect":"Katkaise yhteys","connect_hint":"Chrome näyttää löytämänsä USB-servokortit. Valitse URT-2: Macilla ”USB Single Serial”, Windowsissa ”USB Serial Device (COM…)”.","show_all_ports":"Sovittimeni ei ole listassa","connected":"Yhdistetty: {name}","connect_failed":"Porttia ei voitu avata","disconnected":"Yhteys katkaistu","ready":"Yhdistetty. Valitse ID:t ja paina ”Aloita numerointi”.","ready_det":"Servot voi numeroida pelkällä USB-virralla. Keskittämiseen tarvitaan servojen virtalähde.","step_number":"Anna jokaiselle servolle oma ID","start_id":"Ensimmäinen ID","last_id":"Viimeinen ID","presets":"Valmiit alueet:","force_1m":"Aseta samalla kaikki servot nopeuteen 1 Mbps (suositus)","speak":"Piippaa ja sano numero ääneen","one_at_a_time":"Kytke vain YKSI servo kerrallaan. Kaikkien uusien servojen ID on 1, joten kaksi niistä väylässä vastaisi yhtä aikaa.","btn_start":"Aloita numerointi","btn_stop":"Pysäytä","plug_next":"Kytke yksi servo — siitä tulee ID {n}","plug_next_det":"Tiedonsiirtonopeus (baud rate) tunnistetaan automaattisesti.","remove_first":"Irrota ensin servo {id} ja kytke sitten seuraava","remove_first_det":"Seuraavasta servosta tulee ID {n}.","already":"Tällä servolla on jo ID {id}. Kytke seuraava.","writing":"Kirjoitetaan ID {n}…","done":"ID {n} valmis — irrota servo ja merkitse se","was_id":"oli ID {old}","new_model":"uusi malli: katkaise virta ja kytke se uudelleen, tarkista sitten ketju kerran","finished":"Kaikki valmista, viimeinen ID {end}","finished_det":"Kytke servot ketjuun ja paina ”Tarkista ketju”.","stopped":"Pysäytetty","bad_range":"Ensimmäisen ID:n täytyy olla 0:n ja viimeisen ID:n välillä (enintään 253)","col_new":"Uusi ID","col_was":"Aiempi","col_model":"Malli","col_supply":"Syöttö","supply_v":"Syöttö {v} V","io_lost":"Yhteys URT-2:een katkesi","io_lost_det":"Kytke kortti takaisin ja paina ”Yhdistä URT-2”.","collision":"Näyttää siltä, että kytkettynä on useampi kuin yksi servo","collision_det":"Jätä kytketyksi vain yksi servo.","error":"Jokin meni pieleen: {msg}","error_det":"Tarkista kaapeli; työkalu yrittää uudelleen.","step_check":"Tarkista koko ketju","check_hint":"Kytke kaikki numeroidut servot ketjuun ja skannaa ID:t 0–253 nopeudella 1 Mbps.","btn_check":"Tarkista ketju","scanning":"Skannataan ID:itä 0–253…","found":"Servoja löytyi: {n} (ID {ids})","none_found":"Mikään servo ei vastannut nopeudella 1 Mbps. Tarkista kaapeli tai numeroi servot ensin.","duplicates":"Kahdella tai useammalla servolla on sama ID {ids}. Irrota ne ja numeroi ne uudelleen yksi kerrallaan.","missing":"Ei löytynyt: ID {ids}","chain_ok":"Kaikki servot vastaavat ({n}), ei päällekkäisiä ID:itä","chain_check_list":"Tarkista alla oleva luettelo","step_center":"Keskitä servo (valinnainen)","center_hint":"Ennen kuin kiinnität servovarren tai käsivarren, käännä servo keskiasentoon (2048). Kytke yksi servo ja sen virtalähde. Akseli kääntyy.","btn_center":"Keskitä servo","btn_release":"Vapauta","center_search":"Etsitään servoa…","center_none":"Yksittäistä servoa ei löytynyt. Kytke täsmälleen yksi servo.","center_unsupported":"Keskitys ei ole käytettävissä mallille {model}","centering":"Käännetään servo {id} keskelle…","centered":"Servo {id} on keskitetty ja pitää asentonsa. Kiinnitä servovarsi ja paina sitten ”Vapauta”.","position":"asento {p}","center_power":"Servo saa vain {v} V","center_power_det":"Kytke servojen virtalähde (7,4 V tai 12 V servon mukaan).","released":"Vapautettu","log":"Tekninen loki (tukea varten)","preset_duck":"Microduck","duck_note":"Microduckin nivelten ID:t, kuten HD-1910-kopiot niitä käyttävät. Servot numeroidaan tässä järjestyksessä. Tämä valmis alue rastittaa myös suojauksen: ylivirta- ja ylijännitesuojaus ovat tehtaalta pois päältä. Kun kaikki ovat valmiit, sammuta servot ja kytke ne takaisin päälle, ja paina ”Tarkista ketju” lukeaksesi ID:t ja suojauksen takaisin.","j10":"oikea lonkka, kierto","j11":"oikea lonkka, sivuttain","j12":"oikea lonkka, eteen/taakse","j13":"oikea polvi","j14":"oikea nilkka","j20":"vasen lonkka, kierto","j21":"vasen lonkka, sivuttain","j22":"vasen lonkka, eteen/taakse","j23":"vasen polvi","j24":"vasen nilkka","j30":"kaula, ylös/alas","j31":"pää, ylös/alas","j32":"pää, kierto","j33":"pää, kallistus","j34":"nokka","protect":"Kytke ylivirta- ja ylijännitesuojaus päälle (tehtaalta pois; suositus HD-1910:lle)","protect_done":"ylivirta- ja ylijännitesuojaus päällä","protect_skipped":"suojausta ei muutettu: ei saatavilla tässä mallissa","col_prot":"Suojaus","prot_cell":"virta {oc} · jännite {ov}","joint_is":"ID {n} = {joint}"},"fr":{"start_here":"Connectez votre carte URT-2 pour commencer","start_here_det":"Branchez l’URT-2 sur cet ordinateur avec son câble USB, puis cliquez sur « Connecter l’URT-2 ».","unsupported":"Ce navigateur ne peut pas communiquer avec les appareils USB. Ouvrez cette page dans Google Chrome ou Microsoft Edge sur un ordinateur Windows, Mac, Linux ou ChromeOS.","unsupported_short":"Utilisez Chrome ou Edge sur un ordinateur","step_connect":"Connecter l’URT-2","demo_badge":"Démo : servos simulés","btn_connect":"Connecter l’URT-2","btn_disconnect":"Déconnecter","connect_hint":"Chrome affiche les cartes servo USB qu’il détecte. Choisissez l’URT-2 : « USB Single Serial » sur Mac, « Périphérique série USB (COM…) » sous Windows.","show_all_ports":"Mon adaptateur n’est pas dans la liste","connected":"Connecté : {name}","connect_failed":"Impossible d’ouvrir le port","disconnected":"Déconnecté","ready":"Connecté. Choisissez les ID et cliquez sur « Lancer la numérotation ».","ready_det":"L’alimentation USB suffit pour numéroter les servos. Pour les centrer, il faut l’alimentation des servos.","step_number":"Attribuer un ID à chaque servo","start_id":"ID de départ","last_id":"Dernier ID","presets":"Préréglages :","force_1m":"Régler aussi chaque servo sur 1 Mbps (recommandé)","speak":"Bip et annonce du numéro","one_at_a_time":"Branchez UN SEUL servo à la fois. Tous les servos neufs ont l’ID 1 : deux servos sur le bus répondraient en même temps.","btn_start":"Lancer la numérotation","btn_stop":"Arrêter","plug_next":"Branchez un servo — il recevra l’ID {n}","plug_next_det":"La vitesse de transmission est détectée automatiquement.","remove_first":"Débranchez d’abord le servo {id}, puis branchez le suivant","remove_first_det":"Le servo suivant recevra l’ID {n}.","already":"Ce servo a déjà reçu l’ID {id}. Branchez le suivant.","writing":"Écriture de l’ID {n}…","done":"ID {n} terminé — débranchez le servo et étiquetez-le","was_id":"ancien ID {old}","new_model":"nouveau modèle : éteignez-le et rallumez-le, puis vérifiez la chaîne une fois","finished":"Terminé jusqu’à l’ID {end}","finished_det":"Reliez les servos en chaîne et cliquez sur « Vérifier la chaîne ».","stopped":"Arrêté","bad_range":"L’ID de départ doit être compris entre 0 et le dernier ID (253 max.)","col_new":"Nouvel ID","col_was":"Ancien","col_model":"Modèle","col_supply":"Alimentation","supply_v":"alimentation {v} V","io_lost":"Connexion à l’URT-2 perdue","io_lost_det":"Rebranchez la carte et cliquez sur « Connecter l’URT-2 ».","collision":"Plusieurs servos semblent connectés","collision_det":"Ne laissez qu’un seul servo branché.","error":"Un problème est survenu : {msg}","error_det":"Vérifiez le câble ; l’outil va réessayer.","step_check":"Vérifier toute la chaîne","check_hint":"Reliez tous les servos numérotés en chaîne et scannez les ID 0–253 à 1 Mbps.","btn_check":"Vérifier la chaîne","scanning":"Scan des ID 0–253…","found":"Servos trouvés : {n} (ID {ids})","none_found":"Aucun servo n’a répondu à 1 Mbps. Vérifiez le câble ou numérotez d’abord les servos.","duplicates":"Plusieurs servos partagent l’ID {ids}. Débranchez-les et renumérotez-les un par un.","missing":"Introuvable : ID {ids}","chain_ok":"Tous les servos répondent ({n}), aucun doublon","chain_check_list":"Vérifiez la liste ci-dessous","step_center":"Centrer un servo (facultatif)","center_hint":"Avant de monter un palonnier ou un bras, placez le servo en position médiane (2048). Branchez un seul servo et son alimentation. L’axe va tourner.","btn_center":"Centrer le servo","btn_release":"Relâcher","center_search":"Recherche du servo…","center_none":"Aucun servo isolé détecté. Branchez exactement un servo.","center_unsupported":"Le centrage n’est pas disponible pour {model}","centering":"Rotation du servo {id} vers le milieu…","centered":"Le servo {id} est centré et maintient sa position. Montez le palonnier, puis cliquez sur « Relâcher ».","position":"position {p}","center_power":"Le servo ne reçoit que {v} V","center_power_det":"Branchez l’alimentation des servos (7,4 V ou 12 V, selon votre servo).","released":"Relâché","log":"Journal technique (pour le support)","preset_duck":"Microduck","duck_note":"ID des articulations du Microduck, telles qu'utilisées par les répliques à HD-1910. Les servos sont numérotés dans cet ordre. Le préréglage coche aussi la case de protection : les protections contre la surintensité et la surtension sont désactivées en usine. Une fois tous les servos faits, éteignez-les puis rallumez-les, et cliquez sur « Vérifier la chaîne » pour relire les ID et la protection.","j10":"hanche droite, rotation","j11":"hanche droite, latérale","j12":"hanche droite, avant/arrière","j13":"genou droit","j14":"cheville droite","j20":"hanche gauche, rotation","j21":"hanche gauche, latérale","j22":"hanche gauche, avant/arrière","j23":"genou gauche","j24":"cheville gauche","j30":"cou, haut/bas","j31":"tête, haut/bas","j32":"tête, rotation","j33":"tête, inclinaison","j34":"bec","protect":"Activer la protection contre la surintensité et la surtension (désactivée en usine ; recommandé pour le HD-1910)","protect_done":"protection surintensité et surtension activée","protect_skipped":"protection inchangée : non disponible sur ce modèle","col_prot":"Protection","prot_cell":"courant {oc} · tension {ov}","joint_is":"ID {n} = {joint}"},"hu":{"start_here":"A kezdéshez csatlakoztassa az URT-2 panelt","start_here_det":"Csatlakoztassa az URT-2-t ehhez a számítógéphez az USB-kábelével, majd kattintson az „URT-2 csatlakoztatása” gombra.","unsupported":"Ez a böngésző nem tud USB-eszközökkel kommunikálni. Nyissa meg ezt az oldalt Google Chrome-ban vagy Microsoft Edge-ben egy Windows, Mac, Linux vagy ChromeOS rendszerű számítógépen.","unsupported_short":"Használjon Chrome-ot vagy Edge-et számítógépen","step_connect":"Az URT-2 csatlakoztatása","demo_badge":"Demó: szimulált szervók","btn_connect":"URT-2 csatlakoztatása","btn_disconnect":"Leválasztás","connect_hint":"A Chrome felsorolja a talált USB-s szervópaneleket. Válassza az URT-2-t: Macen „USB Single Serial”, Windowson „USB Serial Device (COM…)”.","show_all_ports":"Az adapterem nincs a listában","connected":"Csatlakoztatva: {name}","connect_failed":"Nem sikerült megnyitni a portot","disconnected":"Leválasztva","ready":"Csatlakoztatva. Állítsa be az ID-tartományt, és kattintson a „Számozás indítása” gombra.","ready_det":"A szervók számozásához elég az USB-s táp. A középre állításhoz a szervók tápegysége is kell.","step_number":"Adjon minden szervónak saját ID-t","start_id":"Kezdő ID","last_id":"Utolsó ID","presets":"Előbeállítások:","force_1m":"Minden szervó beállítása 1 Mbps sebességre is (ajánlott)","speak":"Sípolás és a szám bemondása","one_at_a_time":"Egyszerre csak EGY szervót csatlakoztasson. Minden új szervó gyárilag az 1-es ID-t kapja, így ha kettő van a buszon, egyszerre válaszolnának.","btn_start":"Számozás indítása","btn_stop":"Leállítás","plug_next":"Csatlakoztasson egy szervót – az ID-ja {n} lesz","plug_next_det":"Az adatsebességet az eszköz automatikusan felismeri, bármekkora is.","remove_first":"Előbb húzza ki a szervót (ID {id}), majd csatlakoztassa a következőt","remove_first_det":"A következő szervó ID-ja {n} lesz.","already":"Ez a szervó már megkapta az ID-ját ({id}). Csatlakoztassa a következőt.","writing":"ID írása: {n}…","done":"Kész: ID {n} – húzza ki, és címkézze fel","was_id":"korábban ID {old}","new_model":"új modell: kapcsolja ki és be a tápot, majd egyszer ellenőrizze a láncot","finished":"Minden kész (utolsó ID: {end})","finished_det":"Fűzze sorba a szervókat, és kattintson a „Lánc ellenőrzése” gombra.","stopped":"Leállítva","bad_range":"A kezdő ID 0 és az utolsó ID között legyen (max. 253)","col_new":"Új ID","col_was":"Korábbi","col_model":"Modell","col_supply":"Táp","supply_v":"{v} V-os táp","io_lost":"Megszakadt a kapcsolat az URT-2-vel","io_lost_det":"Csatlakoztassa újra, és kattintson az „URT-2 csatlakoztatása” gombra.","collision":"Úgy tűnik, egynél több szervó van csatlakoztatva","collision_det":"Csak egy szervó maradjon csatlakoztatva.","error":"Hiba történt: {msg}","error_det":"Ellenőrizze a kábelt; az eszköz újra megpróbálja.","step_check":"A teljes lánc ellenőrzése","check_hint":"Fűzze sorba az összes megszámozott szervót, és az eszköz 1 Mbps sebességen végigkeresi a 0–253 ID-tartományt.","btn_check":"Lánc ellenőrzése","scanning":"Keresés a 0–253 ID-tartományban…","found":"{n} szervó található: ID {ids}","none_found":"1 Mbps sebességen egyetlen szervó sem válaszolt. Ellenőrizze a kábelt, vagy előbb számozza meg a szervókat.","duplicates":"Két vagy több szervónak ugyanaz az ID-ja: {ids}. Húzza ki őket, és számozza meg újra egyenként.","missing":"Nem található: ID {ids}","chain_ok":"Minden szervó válaszol ({n}), nincs ismétlődés","chain_check_list":"Ellenőrizze az alábbi listát","step_center":"Szervó középre állítása (nem kötelező)","center_hint":"Mielőtt felszereli a szervótárcsát vagy egy kart, forgassa a szervót középállásba (2048). Csatlakoztasson egy szervót és a tápegységét. A tengely el fog fordulni.","btn_center":"Szervó középre állítása","btn_release":"Elengedés","center_search":"Szervó keresése…","center_none":"Nem található pontosan egy szervó. Csak egy szervót csatlakoztasson.","center_unsupported":"A középre állítás ennél a típusnál nem érhető el: {model}","centering":"A szervó (ID {id}) középre forgatása…","centered":"A szervó (ID {id}) középen van, és tartja a helyzetét. Szerelje fel a tárcsát, majd kattintson az „Elengedés” gombra.","position":"pozíció: {p}","center_power":"A szervó csak {v} V-ot kap","center_power_det":"Csatlakoztassa a szervó tápegységét (7,4 V vagy 12 V, ahogy a szervó igényli).","released":"Elengedve","log":"Technikai napló (az ügyfélszolgálatnak)","preset_duck":"Microduck","duck_note":"A Microduck ízületeinek ID-i, ahogy a HD-1910-es replikák használják. A szervók ebben a sorrendben kapnak számot. Az előbeállítás a védelmet is bepipálja: a túláram- és túlfeszültség-védelem gyárilag ki van kapcsolva. Ha mind kész, kapcsolja ki és be a szervókat, majd kattintson a „Lánc ellenőrzése” gombra az ID-k és a védelem visszaolvasásához.","j10":"jobb csípő, forgatás","j11":"jobb csípő, oldalra","j12":"jobb csípő, előre/hátra","j13":"jobb térd","j14":"jobb boka","j20":"bal csípő, forgatás","j21":"bal csípő, oldalra","j22":"bal csípő, előre/hátra","j23":"bal térd","j24":"bal boka","j30":"nyak, fel/le","j31":"fej, fel/le","j32":"fej, forgatás","j33":"fej, döntés","j34":"csőr","protect":"Túláram- és túlfeszültség-védelem bekapcsolása (gyárilag ki; HD-1910-hez ajánlott)","protect_done":"túláram- és túlfeszültség-védelem bekapcsolva","protect_skipped":"a védelem nem változott: ennél a modellnél nem érhető el","col_prot":"Védelem","prot_cell":"áram {oc} · feszültség {ov}","joint_is":"ID {n} = {joint}"},"it":{"start_here":"Colleghi la Sua scheda URT-2 per iniziare","start_here_det":"Colleghi la URT-2 a questo computer con il suo cavo USB, poi clicchi su «Collega URT-2».","unsupported":"Questo browser non può comunicare con i dispositivi USB. Apra questa pagina in Google Chrome o Microsoft Edge su un computer Windows, Mac, Linux o ChromeOS.","unsupported_short":"Usi Chrome o Edge su un computer","step_connect":"Collegare la URT-2","demo_badge":"Demo: servo simulati","btn_connect":"Collega URT-2","btn_disconnect":"Disconnetti","connect_hint":"Chrome elenca le schede servo USB che trova. Scelga la URT-2: «USB Single Serial» su Mac, «USB Serial Device (COM…)» su Windows.","show_all_ports":"Il mio adattatore non è nell’elenco","connected":"Connesso: {name}","connect_failed":"Impossibile aprire la porta","disconnected":"Disconnesso","ready":"Connesso. Scelga gli ID e clicchi su «Avvia numerazione».","ready_det":"Per numerare i servo basta l’alimentazione USB. Per centrarli serve l’alimentatore dei servo.","step_number":"Assegnare un ID a ogni servo","start_id":"ID iniziale","last_id":"Ultimo ID","presets":"Preimpostazioni:","force_1m":"Impostare anche tutti i servo a 1 Mbps (consigliato)","speak":"Bip e annuncio vocale del numero","one_at_a_time":"Colleghi UN solo servo alla volta. Tutti i servo nuovi hanno l’ID 1, quindi due servo sul bus risponderebbero insieme.","btn_start":"Avvia numerazione","btn_stop":"Ferma","plug_next":"Colleghi un servo: riceverà l’ID {n}","plug_next_det":"Il baud rate viene rilevato automaticamente.","remove_first":"Scolleghi prima il servo {id}, poi colleghi il successivo","remove_first_det":"Il prossimo servo riceverà l’ID {n}.","already":"Questo servo ha già ricevuto l’ID {id}. Colleghi il successivo.","writing":"Scrittura dell’ID {n}…","done":"ID {n} fatto: scolleghi il servo e gli metta un’etichetta","was_id":"prima ID {old}","new_model":"nuovo modello: lo spenga e lo riaccenda, poi controlli la catena una volta","finished":"Tutto fatto fino all’ID {end}","finished_det":"Colleghi i servo in catena e clicchi su «Controlla la catena».","stopped":"Interrotto","bad_range":"L’ID iniziale deve essere compreso tra 0 e l’ultimo ID (max 253)","col_new":"Nuovo ID","col_was":"Prima","col_model":"Modello","col_supply":"Alimentazione","supply_v":"alimentazione {v} V","io_lost":"Connessione con la URT-2 persa","io_lost_det":"La ricolleghi e clicchi su «Collega URT-2».","collision":"Sembra che sia collegato più di un servo","collision_det":"Lasci collegato un solo servo.","error":"Qualcosa è andato storto: {msg}","error_det":"Controlli il cavo; verrà fatto un nuovo tentativo.","step_check":"Controllare tutta la catena","check_hint":"Colleghi in catena tutti i servo numerati e scansioni gli ID 0–253 a 1 Mbps.","btn_check":"Controlla la catena","scanning":"Scansione degli ID 0–253…","found":"Servo trovati: {n} (ID {ids})","none_found":"Nessun servo ha risposto a 1 Mbps. Controlli il cavo o numeri prima i servo.","duplicates":"Due o più servo hanno lo stesso ID {ids}. Li scolleghi e li numeri di nuovo uno alla volta.","missing":"Non trovati: ID {ids}","chain_ok":"Rispondono tutti i servo ({n}), nessun duplicato","chain_check_list":"Controlli l’elenco qui sotto","step_center":"Centrare un servo (facoltativo)","center_hint":"Prima di montare una squadretta o un braccio, porti il servo in posizione centrale (2048). Colleghi un servo e il suo alimentatore. L’albero ruoterà.","btn_center":"Centra servo","btn_release":"Rilascia","center_search":"Ricerca del servo…","center_none":"Nessun servo singolo trovato. Colleghi esattamente un servo.","center_unsupported":"La centratura non è disponibile per {model}","centering":"Rotazione del servo {id} al centro…","centered":"Il servo {id} è centrato e mantiene la posizione. Monti la squadretta, poi clicchi su «Rilascia».","position":"posizione {p}","center_power":"Il servo riceve solo {v} V","center_power_det":"Colleghi l’alimentatore dei servo (7,4 V o 12 V, secondo il Suo servo).","released":"Rilasciato","log":"Log tecnico (per l’assistenza)","preset_duck":"Microduck","duck_note":"ID delle articolazioni del Microduck, come usate dalle repliche con HD-1910. I servo vengono numerati in questo ordine. Il preset spunta anche la casella della protezione: le protezioni da sovracorrente e sovratensione sono disattivate di fabbrica. Al termine, spenga e riaccenda i servo e clicchi su «Controlla la catena» per rileggere ID e protezione.","j10":"anca destra, rotazione","j11":"anca destra, laterale","j12":"anca destra, avanti/indietro","j13":"ginocchio destro","j14":"caviglia destra","j20":"anca sinistra, rotazione","j21":"anca sinistra, laterale","j22":"anca sinistra, avanti/indietro","j23":"ginocchio sinistro","j24":"caviglia sinistra","j30":"collo, su/giù","j31":"testa, su/giù","j32":"testa, rotazione","j33":"testa, inclinazione","j34":"becco","protect":"Attivare la protezione da sovracorrente e sovratensione (disattivata di fabbrica; consigliato per HD-1910)","protect_done":"protezione da sovracorrente e sovratensione attiva","protect_skipped":"protezione non modificata: non disponibile su questo modello","col_prot":"Protezione","prot_cell":"corrente {oc} · tensione {ov}","joint_is":"ID {n} = {joint}"},"ja":{"start_here":"URT-2 ボードを接続して始めましょう","start_here_det":"URT-2 を USB ケーブルでこのパソコンにつなぎ、「URT-2 に接続」を押してください。","unsupported":"このブラウザーは USB 機器と通信できません。Windows、Mac、Linux、ChromeOS のパソコンで、Google Chrome または Microsoft Edge を使ってこのページを開いてください。","unsupported_short":"パソコンの Chrome または Edge をお使いください","step_connect":"URT-2 を接続","demo_badge":"デモ：シミュレーションのサーボ","btn_connect":"URT-2 に接続","btn_disconnect":"切断","connect_hint":"見つかった USB サーボボードが Chrome に一覧表示されます。URT-2 を選んでください（Mac では「USB Single Serial」、Windows では「USB シリアル デバイス (COM…)」）。","show_all_ports":"アダプターが一覧にない場合","connected":"接続済み：{name}","connect_failed":"ポートを開けませんでした","disconnected":"切断しました","ready":"接続しました。ID を選んで「番号付けを開始」を押してください。","ready_det":"番号付けは USB の電源だけでできます。中央合わせにはサーボ用電源が必要です。","step_number":"サーボごとに ID を割り当てる","start_id":"開始 ID","last_id":"最後の ID","presets":"プリセット：","force_1m":"すべてのサーボを 1 Mbps にも設定する（推奨）","speak":"ビープ音と音声で番号を知らせる","one_at_a_time":"サーボは必ず1台ずつ接続してください。新品のサーボはすべて ID 1 なので、バスに2台つなぐと同時に応答してしまいます。","btn_start":"番号付けを開始","btn_stop":"停止","plug_next":"サーボを1台接続してください。ID {n} になります","plug_next_det":"通信速度（ボーレート）は自動で検出します。","remove_first":"先にサーボ {id} を外してから、次のサーボを接続してください","remove_first_det":"次のサーボは ID {n} になります。","already":"このサーボはすでに ID {id} です。次のサーボを接続してください。","writing":"ID {n} を書き込み中…","done":"ID {n} 完了：外してラベルを貼ってください","was_id":"元は ID {old}","new_model":"新しいモデル：電源を入れ直してから、一度チェーンを確認してください","finished":"ID {end} まですべて完了しました","finished_det":"サーボを数珠つなぎに接続して「チェーンを確認」を押してください。","stopped":"停止しました","bad_range":"開始 ID は 0 から最後の ID まで（最大 253）の範囲で指定してください","col_new":"新しい ID","col_was":"元の ID","col_model":"モデル","col_supply":"電源","supply_v":"電源 {v} V","io_lost":"URT-2 との接続が切れました","io_lost_det":"もう一度つないで「URT-2 に接続」を押してください。","collision":"複数のサーボが接続されているようです","collision_det":"サーボは1台だけ接続してください。","error":"問題が発生しました：{msg}","error_det":"ケーブルを確認してください。自動で再試行します。","step_check":"チェーン全体を確認","check_hint":"番号を付けたサーボをすべて数珠つなぎに接続し、1 Mbps で ID 0–253 をスキャンします。","btn_check":"チェーンを確認","scanning":"ID 0–253 をスキャン中…","found":"{n}台のサーボが見つかりました：ID {ids}","none_found":"1 Mbps で応答したサーボはありません。ケーブルを確認するか、先にサーボに番号を付けてください。","duplicates":"ID {ids} が複数のサーボで重複しています。外してから1台ずつ番号を付け直してください。","missing":"見つかりません：ID {ids}","chain_ok":"{n}台すべてが応答し、重複はありません","chain_check_list":"下の一覧を確認してください","step_center":"サーボを中央に合わせる（任意）","center_hint":"ホーンやアームを取り付ける前に、サーボを中央位置（2048）まで回します。サーボ1台とサーボ用電源を接続してください。軸が回転します。","btn_center":"中央に合わせる","btn_release":"脱力","center_search":"サーボを探しています…","center_none":"サーボが見つからないか、複数つながっています。サーボを1台だけ接続してください。","center_unsupported":"{model} は中央合わせに対応していません","centering":"サーボ {id} を中央に回しています…","centered":"サーボ {id} は中央位置で保持しています。ホーンを取り付けてから「脱力」を押してください。","position":"位置 {p}","center_power":"サーボへの電源が {v} V しかありません","center_power_det":"サーボ用電源（サーボに合わせて 7.4 V または 12 V）を接続してください。","released":"脱力しました","log":"技術ログ（サポート用）","preset_duck":"Microduck","duck_note":"HD-1910 で作る Microduck レプリカの関節 ID です。この順番で番号を付けます。このプリセットは保護のチェックも入れます（過電流保護と過電圧保護は出荷時オフ）。すべて終わったら、サーボの電源を一度切って入れ直し、「チェーンを確認」を押して ID と保護設定を読み戻してください。","j10":"右股関節・ひねり","j11":"右股関節・左右","j12":"右股関節・前後","j13":"右ひざ","j14":"右足首","j20":"左股関節・ひねり","j21":"左股関節・左右","j22":"左股関節・前後","j23":"左ひざ","j24":"左足首","j30":"首・上下","j31":"頭・上下","j32":"頭・左右回転","j33":"頭・傾き","j34":"くちばし","protect":"過電流保護と過電圧保護をオンにする（出荷時オフ、HD-1910 に推奨）","protect_done":"過電流保護と過電圧保護をオン","protect_skipped":"保護は未変更（この型番では使えません）","col_prot":"保護","prot_cell":"電流 {oc}・電圧 {ov}","joint_is":"ID {n} = {joint}"},"ko":{"start_here":"URT-2 보드를 연결해 시작하세요","start_here_det":"URT-2를 USB 케이블로 이 컴퓨터에 연결한 뒤 “URT-2 연결”을 누르세요.","unsupported":"이 브라우저는 USB 장치와 통신할 수 없습니다. Windows, Mac, Linux 또는 ChromeOS 컴퓨터에서 Google Chrome이나 Microsoft Edge로 이 페이지를 여세요.","unsupported_short":"컴퓨터에서 Chrome 또는 Edge를 사용하세요","step_connect":"URT-2 연결","demo_badge":"데모: 가상 서보","btn_connect":"URT-2 연결","btn_disconnect":"연결 해제","connect_hint":"Chrome이 찾은 USB 서보 보드 목록이 표시됩니다. URT-2를 선택하세요(Mac에서는 “USB Single Serial”, Windows에서는 “USB 직렬 장치(COM…)”).","show_all_ports":"어댑터가 목록에 없나요?","connected":"연결됨: {name}","connect_failed":"포트를 열 수 없습니다","disconnected":"연결이 해제되었습니다","ready":"연결되었습니다. ID를 고르고 “번호 매기기 시작”을 누르세요.","ready_det":"USB 전원만으로도 서보 번호를 매길 수 있습니다. 중앙 정렬에는 서보 전원이 필요합니다.","step_number":"서보마다 ID 지정","start_id":"시작 ID","last_id":"마지막 ID","presets":"프리셋:","force_1m":"모든 서보를 1 Mbps로도 설정(권장)","speak":"삐 소리와 함께 번호 읽어 주기","one_at_a_time":"서보는 반드시 한 번에 하나씩만 연결하세요. 새 서보는 모두 ID 1이라서 버스에 두 개가 있으면 동시에 응답합니다.","btn_start":"번호 매기기 시작","btn_stop":"중지","plug_next":"서보 하나를 연결하세요. ID {n}번이 됩니다","plug_next_det":"통신 속도(보드레이트)는 자동으로 찾습니다.","remove_first":"먼저 {id}번 서보를 분리한 뒤 다음 서보를 연결하세요","remove_first_det":"다음 서보는 ID {n}번이 됩니다.","already":"이 서보는 이미 ID {id}번입니다. 다음 서보를 연결하세요.","writing":"ID {n}번 쓰는 중…","done":"ID {n}번 완료: 분리하고 라벨을 붙이세요","was_id":"이전 ID {old}","new_model":"새 모델: 전원을 껐다 켠 뒤 체인을 한 번 확인하세요","finished":"ID {end}번까지 모두 완료했습니다","finished_det":"서보를 체인으로 연결하고 “체인 확인”을 누르세요.","stopped":"중지되었습니다","bad_range":"시작 ID는 0부터 마지막 ID(최대 253) 사이여야 합니다","col_new":"새 ID","col_was":"이전 ID","col_model":"모델","col_supply":"전원","supply_v":"전원 {v} V","io_lost":"URT-2와의 연결이 끊어졌습니다","io_lost_det":"다시 연결하고 “URT-2 연결”을 누르세요.","collision":"서보가 두 개 이상 연결된 것 같습니다","collision_det":"서보를 하나만 연결해 두세요.","error":"문제가 발생했습니다: {msg}","error_det":"케이블을 확인하세요. 자동으로 다시 시도합니다.","step_check":"전체 체인 확인","check_hint":"번호를 매긴 서보를 모두 체인으로 연결하고 1 Mbps에서 ID 0–253을 스캔합니다.","btn_check":"체인 확인","scanning":"ID 0–253 스캔 중…","found":"서보 {n}개를 찾았습니다: ID {ids}","none_found":"1 Mbps에서 응답한 서보가 없습니다. 케이블을 확인하거나 먼저 서보 번호를 매기세요.","duplicates":"ID {ids}: 서보 두 개 이상이 같은 ID를 쓰고 있습니다. 분리한 뒤 하나씩 다시 번호를 매기세요.","missing":"찾지 못함: ID {ids}","chain_ok":"서보 {n}개가 모두 응답하며 중복이 없습니다","chain_check_list":"아래 목록을 확인하세요","step_center":"서보 중앙 정렬(선택)","center_hint":"혼이나 암을 끼우기 전에 서보를 가운데 위치(2048)로 돌려 두세요. 서보 하나와 서보 전원을 연결하세요. 축이 회전합니다.","btn_center":"서보 중앙 정렬","btn_release":"토크 해제","center_search":"서보를 찾는 중…","center_none":"서보가 없거나 여러 개 연결되어 있습니다. 서보를 정확히 하나만 연결하세요.","center_unsupported":"{model} 모델은 중앙 정렬을 지원하지 않습니다","centering":"{id}번 서보를 가운데로 돌리는 중…","centered":"{id}번 서보가 가운데에서 위치를 유지하고 있습니다. 혼을 끼운 뒤 “토크 해제”를 누르세요.","position":"위치 {p}","center_power":"서보 전원이 {v} V밖에 되지 않습니다","center_power_det":"서보 전원(서보에 맞게 7.4 V 또는 12 V)을 연결하세요.","released":"토크가 해제되었습니다","log":"기술 로그(지원용)","preset_duck":"Microduck","duck_note":"HD-1910으로 만드는 Microduck 레플리카의 관절 ID입니다. 이 순서대로 번호를 매깁니다. 이 프리셋은 보호 설정도 체크합니다(과전류·과전압 보호는 출고 시 꺼져 있음). 모두 끝나면 서보 전원을 껐다 켠 뒤 “체인 확인”을 눌러 ID와 보호 설정을 다시 읽으세요.","j10":"오른쪽 고관절 회전","j11":"오른쪽 고관절 좌우","j12":"오른쪽 고관절 앞뒤","j13":"오른쪽 무릎","j14":"오른쪽 발목","j20":"왼쪽 고관절 회전","j21":"왼쪽 고관절 좌우","j22":"왼쪽 고관절 앞뒤","j23":"왼쪽 무릎","j24":"왼쪽 발목","j30":"목 상하","j31":"머리 상하","j32":"머리 회전","j33":"머리 기울기","j34":"부리","protect":"과전류·과전압 보호 켜기(출고 시 꺼짐, HD-1910에 권장)","protect_done":"과전류·과전압 보호 켜짐","protect_skipped":"보호 설정 변경 안 함: 이 모델에서는 사용할 수 없음","col_prot":"보호","prot_cell":"전류 {oc} · 전압 {ov}","joint_is":"ID {n} = {joint}"},"nb":{"start_here":"Koble til URT-2-kortet for å starte","start_here_det":"Koble URT-2 til denne datamaskinen med USB-kabelen, og trykk deretter på «Koble til URT-2».","unsupported":"Denne nettleseren kan ikke kommunisere med USB-enheter. Åpne denne siden i Google Chrome eller Microsoft Edge på en Windows-, Mac-, Linux- eller ChromeOS-datamaskin.","unsupported_short":"Bruk Chrome eller Edge på en datamaskin","step_connect":"Koble til URT-2","demo_badge":"Demo: simulerte servoer","btn_connect":"Koble til URT-2","btn_disconnect":"Koble fra","connect_hint":"Chrome viser USB-servokortene den finner. Velg URT-2: «USB Single Serial» på Mac, «USB Serial Device (COM…)» på Windows.","show_all_ports":"Adapteren min står ikke i listen","connected":"Tilkoblet: {name}","connect_failed":"Kunne ikke åpne porten","disconnected":"Frakoblet","ready":"Tilkoblet. Velg ID-ene og trykk på «Start nummerering».","ready_det":"Du kan nummerere servoer med bare strøm fra USB. Sentrering krever strømforsyningen til servoene.","step_number":"Gi hver servo sin ID","start_id":"Start på ID","last_id":"Siste ID","presets":"Forvalg:","force_1m":"Sett også alle servoer til 1 Mbps (anbefalt)","speak":"Pip og si nummeret","one_at_a_time":"Koble til ÉN servo om gangen. Alle nye servoer leveres med ID 1, så to av dem på bussen ville svart samtidig.","btn_start":"Start nummerering","btn_stop":"Stopp","plug_next":"Koble til én servo – den blir ID {n}","plug_next_det":"Baudraten finnes automatisk, uansett hva den er.","remove_first":"Koble fra servo {id} først, og koble deretter til den neste","remove_first_det":"Neste servo blir ID {n}.","already":"Denne servoen har allerede fått ID {id}. Koble til den neste.","writing":"Skriver ID {n} …","done":"ID {n} ferdig – koble den fra og merk den","was_id":"var ID {old}","new_model":"ny modell: slå strømmen av og på, og sjekk kjeden én gang","finished":"Ferdig til og med ID {end}","finished_det":"Koble servoene sammen i en kjede og trykk på «Sjekk kjeden».","stopped":"Stoppet","bad_range":"Start-ID må være mellom 0 og siste ID (maks. 253)","col_new":"Ny ID","col_was":"Før","col_model":"Modell","col_supply":"Forsyning","supply_v":"{v} V forsyning","io_lost":"Mistet forbindelsen til URT-2","io_lost_det":"Koble den til igjen og trykk på «Koble til URT-2».","collision":"Det ser ut til at mer enn én servo er tilkoblet","collision_det":"La bare én servo være tilkoblet.","error":"Noe gikk galt: {msg}","error_det":"Sjekk kabelen; verktøyet prøver igjen.","step_check":"Sjekk hele kjeden","check_hint":"Koble alle nummererte servoer sammen i en kjede og søk gjennom ID 0–253 på 1 Mbps.","btn_check":"Sjekk kjeden","scanning":"Søker gjennom ID 0–253 …","found":"Servoer funnet: {n} (ID {ids})","none_found":"Ingen servo svarte på 1 Mbps. Sjekk kabelen, eller nummerer servoene først.","duplicates":"To eller flere servoer har samme ID {ids}. Koble dem fra og nummerer dem på nytt, én om gangen.","missing":"Ikke funnet: ID {ids}","chain_ok":"Alle servoene svarer ({n}), ingen duplikater","chain_check_list":"Sjekk listen nedenfor","step_center":"Sentrer en servo (valgfritt)","center_hint":"Før du monterer et servohorn eller en arm, må du dreie servoen til midtstillingen (2048). Koble til én servo og strømforsyningen til den. Akselen kommer til å dreie.","btn_center":"Sentrer servo","btn_release":"Slipp","center_search":"Leter etter servoen …","center_none":"Fant ikke nøyaktig én servo. Koble til bare én servo.","center_unsupported":"Sentrering er ikke tilgjengelig for {model}","centering":"Dreier servo {id} til midten …","centered":"Servo {id} er sentrert og holder posisjonen. Monter hornet, og trykk deretter på «Slipp».","position":"posisjon {p}","center_power":"Servoen får bare {v} V","center_power_det":"Koble til strømforsyningen til servoen (7,4 V eller 12 V, alt etter hva servoen trenger).","released":"Sluppet","log":"Teknisk logg (for kundestøtte)","preset_duck":"Microduck","duck_note":"Microduck-ledd-ID-er, slik HD-1910-replikaene bruker dem. Servoene nummereres i denne rekkefølgen. Forvalget krysser også av for beskyttelsen: overstrøms- og overspenningsvern er slått av fra fabrikken. Når alle er ferdige, slår du servoene av og på igjen og trykker på «Sjekk kjeden» for å lese tilbake ID-ene og beskyttelsen.","j10":"høyre hofte, vri","j11":"høyre hofte, sidelengs","j12":"høyre hofte, frem/tilbake","j13":"høyre kne","j14":"høyre ankel","j20":"venstre hofte, vri","j21":"venstre hofte, sidelengs","j22":"venstre hofte, frem/tilbake","j23":"venstre kne","j24":"venstre ankel","j30":"nakke, opp/ned","j31":"hode, opp/ned","j32":"hode, vri","j33":"hode, vipp","j34":"nebb","protect":"Slå på overstrøms- og overspenningsvern (av fra fabrikken; anbefalt for HD-1910)","protect_done":"overstrøms- og overspenningsvern på","protect_skipped":"beskyttelse ikke endret: finnes ikke på denne modellen","col_prot":"Beskyttelse","prot_cell":"strøm {oc} · spenning {ov}","joint_is":"ID {n} = {joint}"},"nl":{"start_here":"Verbind uw URT-2-board om te beginnen","start_here_det":"Sluit de URT-2 met de USB-kabel aan op deze computer en klik dan op “URT-2 verbinden”.","unsupported":"Deze browser kan niet met USB-apparaten communiceren. Open deze pagina in Google Chrome of Microsoft Edge op een Windows-, Mac-, Linux- of ChromeOS-computer.","unsupported_short":"Gebruik Chrome of Edge op een computer","step_connect":"De URT-2 verbinden","demo_badge":"Demo: gesimuleerde servo's","btn_connect":"URT-2 verbinden","btn_disconnect":"Verbinding verbreken","connect_hint":"Chrome toont de USB-servoboards die het vindt. Kies de URT-2: “USB Single Serial” op een Mac, “USB Serial Device (COM…)” op Windows.","show_all_ports":"Mijn adapter staat niet in de lijst","connected":"Verbonden: {name}","connect_failed":"Kan de poort niet openen","disconnected":"Verbinding verbroken","ready":"Verbonden. Kies de ID's en klik op “Nummeren starten”.","ready_det":"Voor het nummeren is stroom via USB genoeg. Voor het centreren is de voeding van de servo's nodig.","step_number":"Elke servo een eigen ID geven","start_id":"Begin-ID","last_id":"Laatste ID","presets":"Voorinstellingen:","force_1m":"Ook elke servo op 1 Mbps zetten (aanbevolen)","speak":"Piepen en het nummer uitspreken","one_at_a_time":"Sluit ÉÉN servo tegelijk aan. Alle nieuwe servo's hebben ID 1, dus twee servo's op de bus zouden tegelijk antwoorden.","btn_start":"Nummeren starten","btn_stop":"Stoppen","plug_next":"Sluit één servo aan – die krijgt ID {n}","plug_next_det":"De baudrate wordt automatisch gevonden.","remove_first":"Koppel eerst servo {id} los en sluit dan de volgende aan","remove_first_det":"De volgende servo krijgt ID {n}.","already":"Deze servo heeft al ID {id}. Sluit de volgende aan.","writing":"ID {n} wordt geschreven…","done":"ID {n} klaar – koppel de servo los en zet er een label op","was_id":"was ID {old}","new_model":"nieuw model: zet hem uit en weer aan en controleer daarna één keer de keten","finished":"Alles klaar tot en met ID {end}","finished_det":"Sluit de servo's in een keten aan en klik op “Keten controleren”.","stopped":"Gestopt","bad_range":"De begin-ID moet tussen 0 en de laatste ID liggen (max. 253)","col_new":"Nieuwe ID","col_was":"Was","col_model":"Model","col_supply":"Voeding","supply_v":"{v} V voeding","io_lost":"Verbinding met de URT-2 verloren","io_lost_det":"Sluit hem opnieuw aan en klik op “URT-2 verbinden”.","collision":"Er lijkt meer dan één servo aangesloten te zijn","collision_det":"Laat maar één servo aangesloten.","error":"Er ging iets mis: {msg}","error_det":"Controleer de kabel; er wordt opnieuw geprobeerd.","step_check":"De hele keten controleren","check_hint":"Sluit alle genummerde servo's in een keten aan en scan ID 0–253 op 1 Mbps.","btn_check":"Keten controleren","scanning":"ID 0–253 scannen…","found":"Gevonden servo's: {n} (ID {ids})","none_found":"Geen enkele servo antwoordde op 1 Mbps. Controleer de kabel of nummer eerst de servo's.","duplicates":"Twee of meer servo's delen ID {ids}. Koppel ze los en nummer ze opnieuw, één voor één.","missing":"Niet gevonden: ID {ids}","chain_ok":"Alle servo's antwoorden ({n}), geen dubbele ID's","chain_check_list":"Controleer de lijst hieronder","step_center":"Een servo centreren (optioneel)","center_hint":"Draai de servo naar het midden (2048) voordat u een servohoorn of arm monteert. Sluit één servo en de voeding ervan aan. De as gaat draaien.","btn_center":"Servo centreren","btn_release":"Loslaten","center_search":"Servo zoeken…","center_none":"Geen losse servo gevonden. Sluit precies één servo aan.","center_unsupported":"Centreren is niet beschikbaar voor {model}","centering":"Servo {id} draait naar het midden…","centered":"Servo {id} staat in het midden en houdt die positie vast. Monteer de servohoorn en klik dan op “Loslaten”.","position":"positie {p}","center_power":"De servo krijgt maar {v} V","center_power_det":"Sluit de servovoeding aan (7,4 V of 12 V, afhankelijk van uw servo).","released":"Losgelaten","log":"Technisch logboek (voor support)","preset_duck":"Microduck","duck_note":"Microduck-gewricht-ID's, zoals de HD-1910-replica's ze gebruiken. De servo's worden in deze volgorde genummerd. De voorinstelling vinkt ook de beveiliging aan: overstroom- en overspanningsbeveiliging staan af fabriek uit. Zet daarna alle servo's uit en weer aan en klik op “Keten controleren” om de ID's en de beveiliging terug te lezen.","j10":"rechterheup, draaien","j11":"rechterheup, zijwaarts","j12":"rechterheup, voor/achter","j13":"rechterknie","j14":"rechterenkel","j20":"linkerheup, draaien","j21":"linkerheup, zijwaarts","j22":"linkerheup, voor/achter","j23":"linkerknie","j24":"linkerenkel","j30":"nek, op/neer","j31":"hoofd, op/neer","j32":"hoofd, draaien","j33":"hoofd, kantelen","j34":"snavel","protect":"Overstroom- en overspanningsbeveiliging inschakelen (af fabriek uit; aanbevolen voor HD-1910)","protect_done":"overstroom- en overspanningsbeveiliging aan","protect_skipped":"beveiliging niet gewijzigd: niet beschikbaar op dit model","col_prot":"Beveiliging","prot_cell":"stroom {oc} · spanning {ov}","joint_is":"ID {n} = {joint}"},"pl":{"start_here":"Podłącz płytkę URT-2, aby zacząć","start_here_det":"Podłącz URT-2 do tego komputera kablem USB, a potem kliknij „Połącz URT-2”.","unsupported":"Ta przeglądarka nie obsługuje urządzeń USB. Otwórz tę stronę w Google Chrome lub Microsoft Edge na komputerze z Windows, Linux lub ChromeOS albo na komputerze Mac.","unsupported_short":"Użyj Chrome lub Edge na komputerze","step_connect":"Połącz URT-2","demo_badge":"Demo: symulowane serwa","btn_connect":"Połącz URT-2","btn_disconnect":"Rozłącz","connect_hint":"Chrome pokaże znalezione płytki USB do serw. Wybierz URT-2: „USB Single Serial” na komputerze Mac, „Urządzenie szeregowe USB (COM…)” w Windows.","show_all_ports":"Mojego adaptera nie ma na liście","connected":"Połączono: {name}","connect_failed":"Nie udało się otworzyć portu","disconnected":"Rozłączono","ready":"Połączono. Wybierz ID i kliknij „Rozpocznij numerowanie”.","ready_det":"Do numerowania serw wystarczy zasilanie z USB. Do wyśrodkowania potrzebny jest zasilacz serw.","step_number":"Nadaj każdemu serwu ID","start_id":"Pierwsze ID","last_id":"Ostatnie ID","presets":"Gotowe zakresy:","force_1m":"Ustaw też każde serwo na 1 Mbps (zalecane)","speak":"Sygnał dźwiękowy i numer na głos","one_at_a_time":"Podłączaj JEDNO serwo naraz. Wszystkie nowe serwa mają ID 1, więc dwa z nich na magistrali odpowiadałyby jednocześnie.","btn_start":"Rozpocznij numerowanie","btn_stop":"Zatrzymaj","plug_next":"Podłącz jedno serwo — dostanie ID {n}","plug_next_det":"Prędkość transmisji (baud rate) zostanie wykryta automatycznie.","remove_first":"Najpierw odłącz serwo {id}, potem podłącz następne","remove_first_det":"Następne serwo dostanie ID {n}.","already":"To serwo ma już ID {id}. Podłącz następne.","writing":"Zapisywanie ID {n}…","done":"ID {n} gotowe — odłącz serwo i oznacz je","was_id":"wcześniej ID {old}","new_model":"nowy model: wyłącz i włącz zasilanie, a potem raz sprawdź łańcuch","finished":"Gotowe, aż do ID {end}","finished_det":"Połącz serwa w łańcuch i kliknij „Sprawdź łańcuch”.","stopped":"Zatrzymano","bad_range":"Pierwsze ID musi być między 0 a ostatnim ID (maks. 253)","col_new":"Nowe ID","col_was":"Wcześniej","col_model":"Model","col_supply":"Zasilanie","supply_v":"Zasilanie {v} V","io_lost":"Utracono połączenie z URT-2","io_lost_det":"Podłącz płytkę ponownie i kliknij „Połącz URT-2”.","collision":"Wygląda na to, że podłączono więcej niż jedno serwo","collision_det":"Zostaw podłączone tylko jedno serwo.","error":"Coś poszło nie tak: {msg}","error_det":"Sprawdź kabel; narzędzie spróbuje ponownie.","step_check":"Sprawdź cały łańcuch","check_hint":"Połącz wszystkie ponumerowane serwa w łańcuch i przeskanuj ID 0–253 przy 1 Mbps.","btn_check":"Sprawdź łańcuch","scanning":"Skanowanie ID 0–253…","found":"Znalezione serwa: {n} — ID {ids}","none_found":"Żadne serwo nie odpowiedziało przy 1 Mbps. Sprawdź kabel albo najpierw ponumeruj serwa.","duplicates":"Co najmniej dwa serwa mają to samo ID {ids}. Odłącz je i ponumeruj ponownie, po jednym.","missing":"Nie znaleziono: ID {ids}","chain_ok":"Odpowiadają wszystkie serwa ({n}), bez powtórzonych ID","chain_check_list":"Sprawdź listę poniżej","step_center":"Wyśrodkuj serwo (opcjonalnie)","center_hint":"Zanim założysz orczyk lub ramię, ustaw serwo w pozycji środkowej (2048). Podłącz jedno serwo i jego zasilacz. Wałek się obróci.","btn_center":"Wyśrodkuj serwo","btn_release":"Zwolnij","center_search":"Szukam serwa…","center_none":"Nie znaleziono pojedynczego serwa. Podłącz dokładnie jedno serwo.","center_unsupported":"Wyśrodkowanie nie jest dostępne dla {model}","centering":"Obracanie serwa {id} do środka…","centered":"Serwo {id} jest wyśrodkowane i trzyma pozycję. Załóż orczyk, a potem kliknij „Zwolnij”.","position":"pozycja {p}","center_power":"Serwo dostaje tylko {v} V","center_power_det":"Podłącz zasilacz serw (7,4 V lub 12 V, zależnie od serwa).","released":"Zwolniono","log":"Log techniczny (dla pomocy technicznej)","preset_duck":"Microduck","duck_note":"ID przegubów Microducka, takie jak w replikach na HD-1910. Serwa są numerowane w tej kolejności. Ten zakres zaznacza też pole ochrony: zabezpieczenie nadprądowe i nadnapięciowe jest fabrycznie wyłączone. Po zakończeniu wyłącz i włącz serwa, a potem kliknij „Sprawdź łańcuch”, aby odczytać ID i ochronę.","j10":"prawe biodro, obrót","j11":"prawe biodro, na boki","j12":"prawe biodro, przód/tył","j13":"prawe kolano","j14":"prawa kostka","j20":"lewe biodro, obrót","j21":"lewe biodro, na boki","j22":"lewe biodro, przód/tył","j23":"lewe kolano","j24":"lewa kostka","j30":"szyja, góra/dół","j31":"głowa, góra/dół","j32":"głowa, obrót","j33":"głowa, przechył","j34":"dziób","protect":"Włącz zabezpieczenie nadprądowe i nadnapięciowe (fabrycznie wyłączone; zalecane dla HD-1910)","protect_done":"zabezpieczenie nadprądowe i nadnapięciowe włączone","protect_skipped":"ochrona bez zmian: niedostępna w tym modelu","col_prot":"Ochrona","prot_cell":"prąd {oc} · napięcie {ov}","joint_is":"ID {n} = {joint}"},"pt-PT":{"start_here":"Ligue a placa URT-2 para começar","start_here_det":"Ligue o URT-2 a este computador com o cabo USB e depois clique em «Ligar URT-2».","unsupported":"Este navegador não consegue comunicar com dispositivos USB. Abra esta página no Google Chrome ou no Microsoft Edge num computador com Windows, Mac, Linux ou ChromeOS.","unsupported_short":"Use o Chrome ou o Edge num computador","step_connect":"Ligar o URT-2","demo_badge":"Demonstração: servos simulados","btn_connect":"Ligar URT-2","btn_disconnect":"Desligar","connect_hint":"O Chrome mostra as placas USB de servos que encontrar. Escolha o URT-2: «USB Single Serial» num Mac, «USB Serial Device (COM…)» no Windows.","show_all_ports":"O meu adaptador não aparece na lista","connected":"Ligado: {name}","connect_failed":"Não foi possível abrir a porta","disconnected":"Desligado","ready":"Ligado. Escolha os IDs e clique em «Iniciar numeração».","ready_det":"Pode numerar os servos só com a alimentação USB. Para centrar, precisa da fonte de alimentação dos servos.","step_number":"Dar um ID a cada servo","start_id":"Começar no ID","last_id":"Último ID","presets":"Predefinições:","force_1m":"Configurar também todos os servos para 1 Mbps (recomendado)","speak":"Emitir um sinal sonoro e dizer o número","one_at_a_time":"Ligue UM servo de cada vez. Todos os servos novos vêm com o ID 1, por isso dois deles no barramento responderiam ao mesmo tempo.","btn_start":"Iniciar numeração","btn_stop":"Parar","plug_next":"Ligue um servo — vai ficar com o ID {n}","plug_next_det":"A velocidade de transmissão (baud rate) é detetada automaticamente.","remove_first":"Retire primeiro o servo {id} e depois ligue o seguinte","remove_first_det":"O próximo servo fica com o ID {n}.","already":"Este servo já tem o ID {id}. Ligue o seguinte.","writing":"A gravar o ID {n}…","done":"ID {n} concluído — retire-o e coloque-lhe uma etiqueta","was_id":"era o ID {old}","new_model":"modelo novo: desligue e volte a ligar a alimentação e depois verifique a cadeia uma vez","finished":"Concluído até ao ID {end}","finished_det":"Ligue os servos em cadeia e clique em «Verificar a cadeia».","stopped":"Parado","bad_range":"O ID inicial tem de estar entre 0 e o último ID (máx. 253)","col_new":"ID novo","col_was":"Antes","col_model":"Modelo","col_supply":"Alimentação","supply_v":"Alimentação {v} V","io_lost":"A ligação ao URT-2 perdeu-se","io_lost_det":"Volte a ligá-lo e clique em «Ligar URT-2».","collision":"Parece haver mais do que um servo ligado","collision_det":"Deixe apenas um servo ligado.","error":"Algo correu mal: {msg}","error_det":"Verifique o cabo; a ferramenta vai tentar de novo.","step_check":"Verificar a cadeia completa","check_hint":"Ligue todos os servos numerados em cadeia e pesquise os IDs 0–253 a 1 Mbps.","btn_check":"Verificar a cadeia","scanning":"A pesquisar os IDs 0–253…","found":"Servos encontrados: {n} — ID {ids}","none_found":"Nenhum servo respondeu a 1 Mbps. Verifique o cabo ou numere primeiro os servos.","duplicates":"Dois ou mais servos têm o mesmo ID {ids}. Retire-os e volte a numerá-los um de cada vez.","missing":"Não encontrados: ID {ids}","chain_ok":"Todos os servos respondem ({n}), sem IDs repetidos","chain_check_list":"Verifique a lista abaixo","step_center":"Centrar um servo (opcional)","center_hint":"Antes de montar o horn (disco do servo) ou um braço, rode o servo para a posição central (2048). Ligue um servo e a respetiva fonte de alimentação. O eixo vai rodar.","btn_center":"Centrar servo","btn_release":"Libertar","center_search":"À procura do servo…","center_none":"Não foi encontrado um único servo. Ligue exatamente um servo.","center_unsupported":"A centragem não está disponível para {model}","centering":"A rodar o servo {id} para o centro…","centered":"O servo {id} está centrado e a manter a posição. Monte o horn e depois clique em «Libertar».","position":"posição {p}","center_power":"O servo só recebe {v} V","center_power_det":"Ligue a fonte de alimentação dos servos (7,4 V ou 12 V, conforme o seu servo).","released":"Libertado","log":"Registo técnico (para o suporte)","preset_duck":"Microduck","duck_note":"IDs das articulações do Microduck, tal como usados pelas réplicas com HD-1910. Os servos são numerados por esta ordem. A predefinição também marca a caixa da proteção: as proteções contra sobrecorrente e sobretensão vêm desligadas de fábrica. Quando terminar, desligue e volte a ligar os servos e clique em «Verificar a cadeia» para ler de novo os IDs e a proteção.","j10":"anca direita, rotação","j11":"anca direita, lateral","j12":"anca direita, frente/trás","j13":"joelho direito","j14":"tornozelo direito","j20":"anca esquerda, rotação","j21":"anca esquerda, lateral","j22":"anca esquerda, frente/trás","j23":"joelho esquerdo","j24":"tornozelo esquerdo","j30":"pescoço, cima/baixo","j31":"cabeça, cima/baixo","j32":"cabeça, rotação","j33":"cabeça, inclinação","j34":"bico","protect":"Ligar a proteção contra sobrecorrente e sobretensão (desligada de fábrica; recomendado para HD-1910)","protect_done":"proteção contra sobrecorrente e sobretensão ligada","protect_skipped":"proteção sem alterações: não disponível neste modelo","col_prot":"Proteção","prot_cell":"corrente {oc} · tensão {ov}","joint_is":"ID {n} = {joint}"},"ro":{"start_here":"Conectați placa URT-2 pentru a începe","start_here_det":"Conectați URT-2 la acest computer cu cablul USB, apoi apăsați „Conectare URT-2”.","unsupported":"Acest browser nu poate comunica cu dispozitive USB. Deschideți această pagină în Google Chrome sau Microsoft Edge pe un computer cu Windows, Mac, Linux sau ChromeOS.","unsupported_short":"Folosiți Chrome sau Edge pe un computer","step_connect":"Conectați URT-2","demo_badge":"Demo: servomotoare simulate","btn_connect":"Conectare URT-2","btn_disconnect":"Deconectare","connect_hint":"Chrome afișează plăcile USB pentru servomotoare pe care le găsește. Alegeți URT-2: „USB Single Serial” pe Mac, „USB Serial Device (COM…)” pe Windows.","show_all_ports":"Adaptorul meu nu este în listă","connected":"Conectat: {name}","connect_failed":"Portul nu a putut fi deschis","disconnected":"Deconectat","ready":"Conectat. Alegeți ID-urile și apăsați „Pornire numerotare”.","ready_det":"Puteți numerota servomotoarele doar cu alimentare prin USB. Pentru centrare este nevoie de sursa de alimentare a servomotoarelor.","step_number":"Dați fiecărui servomotor ID-ul său","start_id":"ID de început","last_id":"Ultimul ID","presets":"Presetări:","force_1m":"Setați și toate servomotoarele la 1 Mbps (recomandat)","speak":"Semnal sonor și numărul rostit","one_at_a_time":"Conectați UN SINGUR servomotor odată. Toate servomotoarele noi vin cu ID 1, așa că două pe magistrală ar răspunde simultan.","btn_start":"Pornire numerotare","btn_stop":"Oprire","plug_next":"Conectați un servomotor – va primi ID {n}","plug_next_det":"Viteza de transmisie este detectată automat, oricare ar fi.","remove_first":"Deconectați mai întâi servomotorul {id}, apoi conectați-l pe următorul","remove_first_det":"Următorul servomotor va primi ID {n}.","already":"Acest servomotor a primit deja ID {id}. Conectați-l pe următorul.","writing":"Se scrie ID {n}…","done":"ID {n} gata – deconectați-l și etichetați-l","was_id":"era ID {old}","new_model":"model nou: opriți și reporniți alimentarea, apoi verificați lanțul o dată","finished":"Gata până la ID {end}","finished_det":"Conectați servomotoarele în lanț și apăsați „Verificare lanț”.","stopped":"Oprit","bad_range":"ID-ul de început trebuie să fie între 0 și ultimul ID (max. 253)","col_new":"ID nou","col_was":"Anterior","col_model":"Model","col_supply":"Alimentare","supply_v":"alimentare {v} V","io_lost":"Conexiunea cu URT-2 s-a pierdut","io_lost_det":"Conectați-o din nou și apăsați „Conectare URT-2”.","collision":"Se pare că sunt conectate mai multe servomotoare","collision_det":"Lăsați conectat un singur servomotor.","error":"Ceva nu a mers: {msg}","error_det":"Verificați cablul; se va reîncerca.","step_check":"Verificați întregul lanț","check_hint":"Conectați în lanț toate servomotoarele numerotate și scanați ID-urile 0–253 la 1 Mbps.","btn_check":"Verificare lanț","scanning":"Se scanează ID-urile 0–253…","found":"Servomotoare găsite: {n} (ID {ids})","none_found":"Niciun servomotor nu a răspuns la 1 Mbps. Verificați cablul sau numerotați mai întâi servomotoarele.","duplicates":"Două sau mai multe servomotoare au același ID {ids}. Deconectați-le și numerotați-le din nou, pe rând.","missing":"Lipsă: ID {ids}","chain_ok":"Toate servomotoarele ({n}) răspund, fără duplicate","chain_check_list":"Verificați lista de mai jos","step_center":"Centrați un servomotor (opțional)","center_hint":"Înainte să montați un palonier sau un braț, rotiți servomotorul în poziția de mijloc (2048). Conectați un singur servomotor și sursa lui de alimentare. Axul se va roti.","btn_center":"Centrare servomotor","btn_release":"Eliberare","center_search":"Se caută servomotorul…","center_none":"Nu s-a găsit exact un servomotor. Conectați un singur servomotor.","center_unsupported":"Centrarea nu este disponibilă pentru {model}","centering":"Se rotește servomotorul {id} la mijloc…","centered":"Servomotorul {id} este centrat și își menține poziția. Montați palonierul, apoi apăsați „Eliberare”.","position":"poziția {p}","center_power":"Servomotorul primește doar {v} V","center_power_det":"Conectați sursa de alimentare a servomotorului (7,4 V sau 12 V, după cum are nevoie servomotorul).","released":"Eliberat","log":"Jurnal tehnic (pentru asistență)","preset_duck":"Microduck","duck_note":"ID-urile articulațiilor Microduck, așa cum le folosesc replicile cu HD-1910. Servomotoarele sunt numerotate în această ordine. Presetarea bifează și protecția: protecția la supracurent și la supratensiune este dezactivată din fabrică. Când sunt gata toate, opriți și porniți servomotoarele, apoi apăsați „Verificare lanț” pentru a citi înapoi ID-urile și protecția.","j10":"șold drept, rotire","j11":"șold drept, lateral","j12":"șold drept, față/spate","j13":"genunchi drept","j14":"gleznă dreaptă","j20":"șold stâng, rotire","j21":"șold stâng, lateral","j22":"șold stâng, față/spate","j23":"genunchi stâng","j24":"gleznă stângă","j30":"gât, sus/jos","j31":"cap, sus/jos","j32":"cap, rotire","j33":"cap, înclinare","j34":"cioc","protect":"Activați protecția la supracurent și supratensiune (dezactivată din fabrică; recomandat pentru HD-1910)","protect_done":"protecție la supracurent și supratensiune activă","protect_skipped":"protecție neschimbată: indisponibilă la acest model","col_prot":"Protecție","prot_cell":"curent {oc} · tensiune {ov}","joint_is":"ID {n} = {joint}"},"sk":{"start_here":"Na začiatok pripojte dosku URT-2","start_here_det":"Zapojte URT-2 do tohto počítača káblom USB a potom kliknite na „Pripojiť URT-2“.","unsupported":"Tento prehliadač nevie komunikovať so zariadeniami USB. Otvorte túto stránku v prehliadači Google Chrome alebo Microsoft Edge na počítači so systémom Windows, Linux alebo ChromeOS alebo na počítači Mac.","unsupported_short":"Použite Chrome alebo Edge na počítači","step_connect":"Pripojte URT-2","demo_badge":"Demo: simulované servá","btn_connect":"Pripojiť URT-2","btn_disconnect":"Odpojiť","connect_hint":"Chrome zobrazí nájdené USB dosky pre servá. Vyberte URT-2: na Macu „USB Single Serial“, vo Windows „USB Serial Device (COM…)“.","show_all_ports":"Môj adaptér v zozname nie je","connected":"Pripojené: {name}","connect_failed":"Port sa nepodarilo otvoriť","disconnected":"Odpojené","ready":"Pripojené. Zvoľte ID a kliknite na „Spustiť číslovanie“.","ready_det":"Servá môžete číslovať len s napájaním z USB. Na vycentrovanie potrebujete napájací zdroj pre servá.","step_number":"Prideľte každému servu jeho ID","start_id":"Začať od ID","last_id":"Posledné ID","presets":"Predvoľby:","force_1m":"Nastaviť aj všetky servá na 1 Mbps (odporúčané)","speak":"Pípnuť a vysloviť číslo","one_at_a_time":"Zapájajte vždy LEN JEDNO servo. Všetky nové servá majú ID 1, takže dve servá na zbernici by odpovedali naraz.","btn_start":"Spustiť číslovanie","btn_stop":"Zastaviť","plug_next":"Zapojte jedno servo – dostane ID {n}","plug_next_det":"Prenosovú rýchlosť nástroj zistí sám, nech je akákoľvek.","remove_first":"Najprv odpojte servo {id}, potom zapojte ďalšie","remove_first_det":"Ďalšie servo dostane ID {n}.","already":"Toto servo už dostalo ID {id}. Zapojte ďalšie.","writing":"Zapisovanie ID {n}…","done":"ID {n} hotové – servo odpojte a označte","was_id":"predtým ID {old}","new_model":"nový model: vypnite a zapnite napájanie a potom raz skontrolujte reťaz","finished":"Hotovo až po ID {end}","finished_det":"Zapojte servá do reťaze a kliknite na „Skontrolovať reťaz“.","stopped":"Zastavené","bad_range":"Počiatočné ID musí byť medzi 0 a posledným ID (max. 253)","col_new":"Nové ID","col_was":"Predtým","col_model":"Model","col_supply":"Napájanie","supply_v":"napájanie {v} V","io_lost":"Spojenie s URT-2 sa prerušilo","io_lost_det":"Znova ju zapojte a kliknite na „Pripojiť URT-2“.","collision":"Zdá sa, že je pripojené viac ako jedno servo","collision_det":"Nechajte zapojené len jedno servo.","error":"Niečo sa pokazilo: {msg}","error_det":"Skontrolujte kábel; nástroj to skúsi znova.","step_check":"Skontrolujte celú reťaz","check_hint":"Zapojte všetky očíslované servá do reťaze a prehľadajte ID 0–253 pri 1 Mbps.","btn_check":"Skontrolovať reťaz","scanning":"Prehľadávanie ID 0–253…","found":"Nájdené servá: {n} (ID {ids})","none_found":"Pri 1 Mbps neodpovedalo žiadne servo. Skontrolujte kábel alebo servá najprv očíslujte.","duplicates":"Aspoň dve servá majú rovnaké ID {ids}. Odpojte ich a očíslujte ich znova, vždy po jednom.","missing":"Nenájdené: ID {ids}","chain_ok":"Všetky servá odpovedajú ({n}), bez duplicít","chain_check_list":"Skontrolujte zoznam nižšie","step_center":"Vycentrujte servo (voliteľné)","center_hint":"Skôr než nasadíte páku serva alebo rameno, otočte servo do stredovej polohy (2048). Pripojte jedno servo a jeho napájací zdroj. Hriadeľ sa otočí.","btn_center":"Vycentrovať servo","btn_release":"Uvoľniť","center_search":"Hľadanie serva…","center_none":"Nepodarilo sa nájsť práve jedno servo. Pripojte presne jedno servo.","center_unsupported":"Vycentrovanie nie je pre {model} k dispozícii","centering":"Otáčanie serva {id} do stredu…","centered":"Servo {id} je v strede a drží polohu. Nasaďte páku a potom kliknite na „Uvoľniť“.","position":"poloha {p}","center_power":"Servo dostáva len {v} V","center_power_det":"Pripojte napájací zdroj serva (7,4 V alebo 12 V podľa toho, čo vaše servo potrebuje).","released":"Uvoľnené","log":"Technický protokol (pre podporu)","preset_duck":"Microduck","duck_note":"ID kĺbov Microducku, ako ich používajú repliky s HD-1910. Servá sa číslujú v tomto poradí. Predvoľba zaškrtne aj ochranu: nadprúdová a prepäťová ochrana je z výroby vypnutá. Keď budú hotové všetky, vypnite a zapnite servá a kliknite na „Skontrolovať reťaz“, aby sa ID a ochrana načítali späť.","j10":"pravé bedro, otočenie","j11":"pravé bedro, do strany","j12":"pravé bedro, vpred/vzad","j13":"pravé koleno","j14":"pravý členok","j20":"ľavé bedro, otočenie","j21":"ľavé bedro, do strany","j22":"ľavé bedro, vpred/vzad","j23":"ľavé koleno","j24":"ľavý členok","j30":"krk, hore/dole","j31":"hlava, hore/dole","j32":"hlava, otočenie","j33":"hlava, náklon","j34":"zobák","protect":"Zapnúť nadprúdovú a prepäťovú ochranu (z výroby vypnutá; odporúčané pre HD-1910)","protect_done":"nadprúdová a prepäťová ochrana zapnutá","protect_skipped":"ochrana nezmenená: pri tomto modeli nie je k dispozícii","col_prot":"Ochrana","prot_cell":"prúd {oc} · napätie {ov}","joint_is":"ID {n} = {joint}"},"sv":{"start_here":"Anslut ditt URT-2-kort för att börja","start_here_det":"Koppla in URT-2 i den här datorn med USB-kabeln och klicka sedan på ”Anslut URT-2”.","unsupported":"Den här webbläsaren kan inte kommunicera med USB-enheter. Öppna sidan i Google Chrome eller Microsoft Edge på en dator med Windows, Mac, Linux eller ChromeOS.","unsupported_short":"Använd Chrome eller Edge på en dator","step_connect":"Anslut URT-2","demo_badge":"Demo: simulerade servon","btn_connect":"Anslut URT-2","btn_disconnect":"Koppla från","connect_hint":"Chrome visar de USB-servokort som hittas. Välj URT-2: ”USB Single Serial” på en Mac, ”USB Serial Device (COM…)” i Windows.","show_all_ports":"Min adapter finns inte i listan","connected":"Ansluten: {name}","connect_failed":"Det gick inte att öppna porten","disconnected":"Frånkopplad","ready":"Ansluten. Välj ID:n och klicka på ”Starta numrering”.","ready_det":"Du kan numrera servon med bara ström från USB. För centrering behövs strömförsörjningen till servona.","step_number":"Ge varje servo sitt ID","start_id":"Börja på ID","last_id":"Sista ID","presets":"Förval:","force_1m":"Ställ även in alla servon på 1 Mbps (rekommenderas)","speak":"Pip och läs upp numret","one_at_a_time":"Koppla in ETT servo i taget. Alla nya servon har ID 1, så två av dem på bussen skulle svara samtidigt.","btn_start":"Starta numrering","btn_stop":"Stoppa","plug_next":"Koppla in ett servo — det får ID {n}","plug_next_det":"Överföringshastigheten (baud rate) hittas automatiskt.","remove_first":"Koppla först ur servo {id} och koppla sedan in nästa","remove_first_det":"Nästa servo får ID {n}.","already":"Det här servot har redan fått ID {id}. Koppla in nästa.","writing":"Skriver ID {n}…","done":"ID {n} klart — koppla ur servot och märk det","was_id":"var ID {old}","new_model":"ny modell: bryt strömmen och slå på den igen, kontrollera sedan kedjan en gång","finished":"Klart till och med ID {end}","finished_det":"Koppla ihop servona i en kedja och klicka på ”Kontrollera kedjan”.","stopped":"Stoppad","bad_range":"Start-ID måste vara mellan 0 och sista ID (max 253)","col_new":"Nytt ID","col_was":"Tidigare","col_model":"Modell","col_supply":"Matning","supply_v":"Matning {v} V","io_lost":"Anslutningen till URT-2 bröts","io_lost_det":"Koppla in kortet igen och klicka på ”Anslut URT-2”.","collision":"Det verkar vara mer än ett servo anslutet","collision_det":"Ha bara ett servo inkopplat.","error":"Något gick fel: {msg}","error_det":"Kontrollera kabeln; verktyget försöker igen.","step_check":"Kontrollera hela kedjan","check_hint":"Koppla ihop alla numrerade servon i en kedja och sök igenom ID 0–253 med 1 Mbps.","btn_check":"Kontrollera kedjan","scanning":"Söker igenom ID 0–253…","found":"Hittade servon: {n} (ID {ids})","none_found":"Inget servo svarade på 1 Mbps. Kontrollera kabeln eller numrera servona först.","duplicates":"Två eller fler servon har samma ID {ids}. Koppla ur dem och numrera om dem ett i taget.","missing":"Hittades inte: ID {ids}","chain_ok":"Alla servon svarar ({n}), inga dubbletter","chain_check_list":"Kontrollera listan nedan","step_center":"Centrera ett servo (valfritt)","center_hint":"Innan du monterar ett servohorn eller en arm vrider du servot till mittläget (2048). Anslut ett servo och dess strömförsörjning. Axeln kommer att vrida sig.","btn_center":"Centrera servo","btn_release":"Släpp","center_search":"Letar efter servot…","center_none":"Hittade inte ett enskilt servo. Anslut exakt ett servo.","center_unsupported":"Centrering är inte tillgänglig för {model}","centering":"Vrider servo {id} till mitten…","centered":"Servo {id} är centrerat och håller läget. Montera servohornet och klicka sedan på ”Släpp”.","position":"läge {p}","center_power":"Servot får bara {v} V","center_power_det":"Anslut strömförsörjningen till servot (7,4 V eller 12 V, beroende på vad ditt servo behöver).","released":"Släppt","log":"Teknisk logg (för support)","preset_duck":"Microduck","duck_note":"Microducks led-ID:n, som HD-1910-replikerna använder dem. Servona numreras i den här ordningen. Förvalet kryssar också i skyddsrutan: överströms- och överspänningsskydd är avstängda från fabrik. När alla är klara, stäng av och slå på servona igen och klicka på ”Kontrollera kedjan” för att läsa tillbaka ID:n och skydd.","j10":"höger höft, vridning","j11":"höger höft, i sidled","j12":"höger höft, fram/bak","j13":"höger knä","j14":"höger fotled","j20":"vänster höft, vridning","j21":"vänster höft, i sidled","j22":"vänster höft, fram/bak","j23":"vänster knä","j24":"vänster fotled","j30":"nacke, upp/ner","j31":"huvud, upp/ner","j32":"huvud, vridning","j33":"huvud, lutning","j34":"näbb","protect":"Slå på överströms- och överspänningsskydd (avstängt från fabrik; rekommenderas för HD-1910)","protect_done":"överströms- och överspänningsskydd på","protect_skipped":"skyddet ändrades inte: finns inte på den här modellen","col_prot":"Skydd","prot_cell":"ström {oc} · spänning {ov}","joint_is":"ID {n} = {joint}"}};
/* ArmDojo Servo ID tool — Web Serial link + page UI. Mounts into <div id="armdojo-servo-tool">.
 * Language: data-lang on the container, else <html lang>, else English. Demo (no hardware): add ?servo-demo=1 to the URL. */
(function () {
  'use strict';
  const C = window.ArmDojoServoCore, M = window.ArmDojoServoMock, I18N = window.ArmDojoServoI18n || {};
  const { Bus, Numberer, BusError, compressIds, sleep, now, UNLOAD, MICRODUCK_IDS } = C;
  const PROTECT_BITS = UNLOAD.CURRENT | UNLOAD.VOLTAGE;

  // ---------------- Web Serial link ----------------
  class WebSerialLink {
    constructor(port) { this.port = port; this.baud = 0; this.rx = []; this.waiter = null; this.dead = null; this.name = 'URT-2'; }
    async open(baud) {
      await this.port.open({ baudRate: baud, dataBits: 8, stopBits: 1, parity: 'none', flowControl: 'none', bufferSize: 4096 });
      try { await this.port.setSignals({ dataTerminalReady: true, requestToSend: true }); } catch (_) { /* not all boards */ }
      this.baud = baud;
      this.writer = this.port.writable.getWriter();
      this.reader = this.port.readable.getReader();
      this.loop = this.readLoop(this.reader);
    }
    async readLoop(reader) {
      try {
        for (;;) {
          const { value, done } = await reader.read();
          if (done) break;
          if (value) { for (let i = 0; i < value.length; i++) this.rx.push(value[i]); this.wake(); }
        }
      } catch (e) { this.dead = e; this.wake(); }
    }
    wake() { const w = this.waiter; this.waiter = null; if (w) w(); }
    async shut() {
      try { await this.reader.cancel(); } catch (_) {}
      try { this.reader.releaseLock(); } catch (_) {}
      try { this.writer.releaseLock(); } catch (_) {}
      try { await this.loop; } catch (_) {}
      try { await this.port.close(); } catch (_) {}
    }
    async setBaud(b) {   // Web Serial changes the baud rate by reopening the port
      if (b === this.baud) return;
      await this.shut(); this.rx.length = 0;
      await this.open(b); await sleep(5); this.rx.length = 0;
    }
    flush() { this.rx.length = 0; }
    async write(bytes) {
      if (this.dead) throw new BusError('io', 'URT-2 disconnected');
      try { await this.writer.write(bytes); } catch (e) { this.dead = e; throw new BusError('io', 'URT-2 disconnected'); }
    }
    async read(out, want, deadline) {
      for (;;) {
        if (this.rx.length) { for (let i = 0; i < this.rx.length; i++) out.push(this.rx[i]); this.rx.length = 0; }
        if (this.dead) throw new BusError('io', 'URT-2 disconnected');
        if (out.length >= want) return;
        const left = deadline - now();
        if (left <= 0) return;
        await new Promise((r) => { this.waiter = r; setTimeout(r, left); });
      }
    }
    async close() { await this.shut(); }
  }

  // ---------------- i18n ----------------
  const root = document.getElementById('armdojo-servo-tool');
  if (!root) return;
  const pick = (l) => (l && (I18N[l] || I18N[l.split('-')[0]])) || null;
  const lang = (root.dataset.lang || document.documentElement.lang || 'en');
  const S = Object.assign({}, I18N.en || {}, pick(lang) || {});
  const t = (k, a) => (S[k] || k).replace(/\{(\w+)\}/g, (_, n) => (a && a[n] != null ? a[n] : ''));
  const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
  const v1 = (x) => { try { return Number(x).toLocaleString(lang, { minimumFractionDigits: 1, maximumFractionDigits: 1 }); } catch (_) { return Number(x).toFixed(1); } };

  // ---------------- styles ----------------
  const css = `
#armdojo-servo-tool{--a:#1F5FBF;--ink:#1A2330;--bg:#F5F7FA;--ok:#1d8a4a;--warn:#c77700;--bad:#c62828;font-size:16px;line-height:1.45;color:var(--ink);max-width:880px;margin:1.2em 0 2em}
#armdojo-servo-tool *{box-sizing:border-box}
#armdojo-servo-tool .st{display:flex;gap:16px;align-items:center;background:var(--bg);border:1px solid #dfe5ee;border-radius:14px;padding:16px 18px;margin-bottom:14px}
#armdojo-servo-tool .big{flex:0 0 88px;height:88px;border-radius:50%;display:flex;align-items:center;justify-content:center;font-size:40px;font-weight:700;background:#fff;border:4px solid #c9d3e3;color:var(--ink)}
#armdojo-servo-tool .st.waiting .big{border-color:var(--a);color:var(--a)}
#armdojo-servo-tool .st.working .big{border-color:var(--warn);color:var(--warn)}
#armdojo-servo-tool .st.success .big,#armdojo-servo-tool .st.finished .big{border-color:var(--ok);color:#fff;background:var(--ok)}
#armdojo-servo-tool .st.warning .big{border-color:var(--warn)}
#armdojo-servo-tool .st.error .big{border-color:var(--bad);color:var(--bad)}
#armdojo-servo-tool .msg{font-weight:700;font-size:18px}
#armdojo-servo-tool .det{color:#4a5568;font-size:14px;margin-top:2px}
#armdojo-servo-tool .card{border:1px solid #dfe5ee;border-radius:14px;padding:14px 18px;margin-bottom:14px;background:#fff}
#armdojo-servo-tool .card h3{margin:0 0 10px;font-size:17px}
#armdojo-servo-tool .row{display:flex;flex-wrap:wrap;gap:10px;align-items:center}
#armdojo-servo-tool button{font:inherit;font-weight:600;border-radius:10px;border:1px solid var(--a);background:var(--a);color:#fff;padding:10px 16px;cursor:pointer;min-height:44px}
#armdojo-servo-tool button.sec{background:#fff;color:var(--a)}
#armdojo-servo-tool button.pre{background:#fff;color:var(--ink);border-color:#c9d3e3;padding:6px 10px;min-height:36px;font-weight:500}
#armdojo-servo-tool button:disabled{opacity:.45;cursor:default}
#armdojo-servo-tool input[type=number]{font:inherit;width:84px;padding:8px;border:1px solid #c9d3e3;border-radius:8px}
#armdojo-servo-tool label{display:inline-flex;gap:6px;align-items:center}
#armdojo-servo-tool .muted{color:#4a5568;font-size:14px}
#armdojo-servo-tool table{border-collapse:collapse;width:100%;margin-top:10px;font-size:14px}
#armdojo-servo-tool th,#armdojo-servo-tool td{text-align:left;padding:6px 8px;border-bottom:1px solid #eef1f6}
#armdojo-servo-tool .note{background:#fff8e6;border:1px solid #f3d38a;border-radius:10px;padding:10px 12px;font-size:14px;margin:8px 0}
#armdojo-servo-tool .badge{display:inline-block;background:var(--a);color:#fff;border-radius:6px;padding:2px 8px;font-size:12px;margin-left:8px}
#armdojo-servo-tool details{font-size:13px;color:#4a5568}
#armdojo-servo-tool pre{white-space:pre-wrap;max-height:200px;overflow:auto;background:var(--bg);padding:8px;border-radius:8px}
#armdojo-servo-tool .bar{height:6px;background:#e6ebf2;border-radius:3px;overflow:hidden;margin-top:8px}
#armdojo-servo-tool .bar i{display:block;height:100%;background:var(--a);width:0}
#armdojo-servo-tool button.pre.on{background:var(--a);color:#fff;border-color:var(--a)}
#armdojo-servo-tool .duck{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:2px 14px;margin-top:6px}
#armdojo-servo-tool .duck span b{display:inline-block;min-width:26px}
@media (max-width:600px){#armdojo-servo-tool .duck{grid-template-columns:1fr}}
@media (max-width:600px){#armdojo-servo-tool .big{flex-basis:64px;height:64px;font-size:28px}}`;
  const style = document.createElement('style'); style.textContent = css; document.head.appendChild(style);

  // ---------------- markup ----------------
  const demo = /[?&]servo-demo=1\b/.test(location.search);
  const supported = demo || ('serial' in navigator);
  root.innerHTML = `
<div class="st" data-r="st"><div class="big" data-r="big">ID</div><div><div class="msg" data-r="msg"></div><div class="det" data-r="det"></div><div class="bar" data-r="bar" hidden><i></i></div></div></div>
${supported ? '' : `<div class="note">${esc(t('unsupported'))}</div>`}
<div class="card"><h3>1. ${esc(t('step_connect'))}${demo ? `<span class="badge">${esc(t('demo_badge'))}</span>` : ''}</h3>
  <div class="row"><button data-r="connect">${esc(t('btn_connect'))}</button><button class="sec" data-r="disconnect" hidden>${esc(t('btn_disconnect'))}</button><span class="muted" data-r="port"></span></div>
  <p class="muted" style="margin:8px 0 0">${esc(t('connect_hint'))} <a href="#" data-r="all">${esc(t('show_all_ports'))}</a></p></div>
<div class="card"><h3>2. ${esc(t('step_number'))}</h3>
  <div class="row"><label>${esc(t('start_id'))} <input type="number" min="0" max="253" value="1" data-r="from"></label>
    <label>${esc(t('last_id'))} <input type="number" min="0" max="253" value="6" data-r="to"></label>
    <span class="muted">${esc(t('presets'))}</span><button class="pre" data-p="1-6">1–6</button><button class="pre" data-p="1-12">1–12</button><button class="pre" data-p="1-15">1–15</button><button class="pre" data-p="1-18">1–18</button><button class="pre" data-p="duck">${esc(t('preset_duck'))}</button></div>
  <div class="note" data-r="ducknote" hidden>${esc(t('duck_note'))}<div class="duck">${[10, 20, 30].map((g) => `<div>${[0, 1, 2, 3, 4].map((k) => `<span><b>${g + k}</b> ${esc(t('j' + (g + k)))}</span>`).join('<br>')}</div>`).join('')}</div></div>
  <div class="row" style="margin-top:8px"><label><input type="checkbox" checked data-r="force1m"> ${esc(t('force_1m'))}</label>
    <label><input type="checkbox" data-r="protect"> ${esc(t('protect'))}</label>
    <label><input type="checkbox" checked data-r="speak"> ${esc(t('speak'))}</label></div>
  <div class="note">${esc(t('one_at_a_time'))}</div>
  <div class="row"><button data-r="start" disabled>${esc(t('btn_start'))}</button><button class="sec" data-r="stop" hidden>${esc(t('btn_stop'))}</button></div>
  <table data-r="table" hidden><thead><tr><th>${esc(t('col_new'))}</th><th>${esc(t('col_was'))}</th><th>${esc(t('col_model'))}</th><th>${esc(t('col_supply'))}</th><th>${esc(t('col_prot'))}</th></tr></thead><tbody data-r="rows"></tbody></table></div>
<div class="card"><h3>3. ${esc(t('step_check'))}</h3><p class="muted" style="margin:0 0 8px">${esc(t('check_hint'))}</p>
  <div class="row"><button data-r="check" disabled>${esc(t('btn_check'))}</button></div><div data-r="chain" style="margin-top:8px"></div></div>
<div class="card"><h3>${esc(t('step_center'))}</h3><p class="muted" style="margin:0 0 8px">${esc(t('center_hint'))}</p>
  <div class="row"><button data-r="center" disabled>${esc(t('btn_center'))}</button><button class="sec" data-r="release" hidden>${esc(t('btn_release'))}</button></div></div>
<details><summary>${esc(t('log'))}</summary><pre data-r="log"></pre></details>`;
  const $ = (r) => root.querySelector(`[data-r="${r}"]`);

  // ---------------- state ----------------
  let link = null, bus = null, numberer = null, busy = false, held = null, lastNumbered = null, duckMode = false;
  const done = new Set();
  const joint = (n) => (duckMode && MICRODUCK_IDS.includes(n) ? t('j' + n) : '');
  const prot = (s) => (s && s.unload != null ? t('prot_cell', { oc: s.unload & UNLOAD.CURRENT ? '✓' : '✗', ov: s.unload & UNLOAD.VOLTAGE ? '✓' : '✗' }) : '—');

  const log = (m) => { const p = $('log'); p.textContent += `${new Date().toLocaleTimeString()}  ${m}\n`; p.scrollTop = p.scrollHeight; };
  function status(phase, big, msg, det) {
    $('st').className = 'st ' + (phase || ''); $('big').textContent = big; $('msg').textContent = msg; $('det').textContent = det || '';
  }
  const servoLine = (s) => s ? [s.model, s.voltage != null ? t('supply_v', { v: v1(s.voltage) }) : '', s.baud && s.baud !== 1000000 ? `${s.baud / 1000} kbps` : ''].filter(Boolean).join(' · ') : '';
  function refresh() {
    const on = !!bus, numbering = !!numberer;
    $('connect').hidden = on; $('disconnect').hidden = !on;
    $('start').disabled = !on || numbering || busy; $('stop').hidden = !numbering;
    $('check').disabled = !on || numbering || busy; $('center').disabled = !on || numbering || busy || !!held;
    $('release').hidden = !held; $('connect').disabled = !supported;
  }
  function say(n) {
    if (!$('speak').checked) return;
    try {   // a short beep, then the number in the page language
      const ac = new (window.AudioContext || window.webkitAudioContext)(), o = ac.createOscillator(), g = ac.createGain();
      o.frequency.value = 880; g.gain.value = 0.08; o.connect(g); g.connect(ac.destination); o.start(); o.stop(ac.currentTime + 0.12);
    } catch (_) {}
    try { const u = new SpeechSynthesisUtterance(String(n)); u.lang = lang; speechSynthesis.cancel(); speechSynthesis.speak(u); } catch (_) {}
  }
  async function exclusive(f) {
    if (busy) return; busy = true; refresh();
    try { await f(); } catch (e) { onError(e); } finally { busy = false; refresh(); }
  }
  function onError(e) {
    log('error: ' + (e && e.message));
    if (e && e.kind === 'io') { lost(); return; }
    if (e && e.kind === 'collision') status('error', '!', t('collision'), t('collision_det'));
    else status('error', '!', t('error', { msg: e && e.message }), t('error_det'));
  }
  function lost() {
    status('error', 'USB', t('io_lost'), t('io_lost_det'));
    if (numberer) numberer.stop();
    numberer = null; bus = null; held = null;
    if (link) { try { link.close(); } catch (_) {} } link = null; refresh();
  }

  // ---------------- connect ----------------
  async function connect(all) {
    try {
      let l;
      if (demo) { l = new M.DemoBench(); l.name = t('demo_badge'); }
      else {
        const ports = all ? [] : await navigator.serial.getPorts();
        const port = ports.length === 1 ? ports[0] : await navigator.serial.requestPort(all ? {} : { filters: [
          { usbVendorId: 0x1A86 }, { usbVendorId: 0x10C4 }, { usbVendorId: 0x0403 }, { usbVendorId: 0x067B }] });
        l = new WebSerialLink(port);
        await l.open(1000000);
        const info = port.getInfo ? port.getInfo() : {};
        l.name = info.usbVendorId === 0x1A86 ? 'URT-2' : `USB ${(info.usbVendorId || 0).toString(16)}:${(info.usbProductId || 0).toString(16)}`;
      }
      link = l; bus = new Bus(l);
      $('port').textContent = t('connected', { name: l.name });
      status('', 'ID', t('ready'), t('ready_det'));
      log('connected: ' + l.name);
    } catch (e) {
      if (e && e.name === 'NotFoundError') return;   // the user closed the chooser
      status('error', '!', t('connect_failed'), e && e.message); log('connect failed: ' + (e && e.message));
    } finally { refresh(); }
  }
  if (!demo && supported) navigator.serial.addEventListener('disconnect', (e) => { if (link && e.target === link.port) { link.dead = new Error('unplugged'); lost(); } });

  $('connect').onclick = () => connect(false);
  $('all').onclick = (e) => { e.preventDefault(); if (!bus && supported) connect(true); };
  $('disconnect').onclick = async () => {
    if (numberer) { numberer.stop(); numberer = null; }
    if (held && bus) { try { await bus.release(held); } catch (_) {} held = null; }
    if (link) await link.close(); link = null; bus = null;
    $('port').textContent = ''; status('', 'ID', t('disconnected'), ''); refresh();
  };
  root.querySelectorAll('[data-p]').forEach((b) => {
    b.onclick = () => {
      duckMode = b.dataset.p === 'duck';
      const [a, z] = duckMode ? [MICRODUCK_IDS[0], MICRODUCK_IDS[MICRODUCK_IDS.length - 1]] : b.dataset.p.split('-');
      $('from').value = a; $('to').value = z;
      if (duckMode) $('protect').checked = true;
      $('ducknote').hidden = !duckMode;
      root.querySelectorAll('[data-p]').forEach((x) => x.classList.toggle('on', x === b && duckMode));
    };
  });

  // ---------------- numbering ----------------
  const MSG = {
    plug_next: (a) => [t('plug_next', a), t('plug_next_det')],
    remove_first: (a) => [t('remove_first', a), t('remove_first_det', a)],
    already: (a) => [t('already', a), ''],
    writing: (a) => [t('writing', a), ''],
    done: (a) => [t('done', a), t('was_id', a)],
    done_new_model: (a) => [t('done', a), t('was_id', a) + ' · ' + t('new_model')],
    finished: (a) => [t('finished', a), t('finished_det')],
    io_lost: () => [t('io_lost'), t('io_lost_det')],
    collision: () => [t('collision'), t('collision_det')],
    error: (a) => [t('error', a), t('error_det')],
  };
  $('start').onclick = async () => {
    const from = parseInt($('from').value, 10), to = parseInt($('to').value, 10);
    const ids = duckMode ? MICRODUCK_IDS.filter((id) => id >= from && id <= to) : null;
    if (!(from >= 0 && to <= 253 && from <= to) || (ids && !ids.length)) { status('error', '!', t('bad_range'), ''); return; }
    if (held) { try { await bus.release(held); } catch (_) {} held = null; }
    if (demo && link.setModel) link.setModel(duckMode ? 7946 : null);
    numberer = new Numberer(bus, { next: from, end: to, ids, unloadBits: $('protect').checked ? PROTECT_BITS : 0, force1M: $('force1m').checked, done, lastNumbered }, (s) => {
      const a = s.args || {};
      const [m, d] = (MSG[s.key] || (() => [s.key, '']))(a);
      const j = s.phase !== 'finished' && a.n != null ? joint(a.n) : '';
      const p = s.key.startsWith('done') && s.servo && s.servo.protect != null ? t(s.servo.protect ? 'protect_done' : 'protect_skipped') : '';
      const extra = s.servo ? servoLine(s.servo) : '';
      status(s.phase, s.big, m, [j ? t('joint_is', { n: a.n, joint: j }) : '', d, p, extra].filter(Boolean).join(' · '));
      if (s.key === 'io_lost') lost();
    }, (old, v, next) => {
      lastNumbered = { id: v.id, baud: v.baud };
      $('table').hidden = false;
      const tr = document.createElement('tr');
      const j = joint(v.id);
      tr.innerHTML = `<td><b>${v.id}</b>${j ? ' ' + esc(j) : ''}</td><td>${old}</td><td>${esc(v.model)}</td><td>${v.voltage != null ? v1(v.voltage) + ' V' : ''}</td><td>${esc(prot(v))}</td>`;
      $('rows').prepend(tr);
      $('from').value = next != null ? next : v.id + 1;
      say(v.id); log(`ID ${old} → ${v.id} (${v.model}, ${v.voltage} V, protection register ${v.unload})`);
    });
    refresh();
    const n = numberer;
    await n.run();
    if (numberer === n) numberer = null;
    refresh();
  };
  $('stop').onclick = () => { if (numberer) { numberer.stop(); numberer = null; status('', 'ID', t('stopped'), ''); refresh(); } };

  // ---------------- chain check ----------------
  $('check').onclick = () => exclusive(async () => {
    $('chain').innerHTML = '';
    status('working', '…', t('scanning'), ''); $('bar').hidden = false;
    await bus.setBaud(1000000);
    const r = await bus.scan((p) => { $('bar').firstChild.style.width = Math.round(p * 100) + '%'; });
    $('bar').hidden = true;
    const rows = [];
    for (const id of r.found) {
      let s = null; try { s = await bus.identify(id); } catch (_) {}
      const j = joint(id);
      rows.push(`<tr><td><b>${id}</b>${j ? ' ' + esc(j) : ''}</td><td>${esc(s ? s.model : '?')}</td><td>${s && s.voltage != null ? v1(s.voltage) + ' V' : ''}</td><td>${esc(prot(s))}</td></tr>`);
    }
    const missing = [];
    if (duckMode) { for (const i of MICRODUCK_IDS) if (!r.found.includes(i) && !r.garbled.includes(i)) missing.push(i); }
    else {
      const from = parseInt($('from').value, 10) || 1, to = parseInt($('to').value, 10) || 0;
      const lo = Math.min(from, ...(r.found.length ? r.found : [from])), hi = Math.max(to, ...(r.found.length ? r.found : [to]));
      for (let i = Math.max(lo, 1); i <= hi; i++) if (!r.found.includes(i) && !r.garbled.includes(i)) missing.push(i);
    }
    let html = r.found.length ? `<p><b>${esc(t('found', { n: r.found.length, ids: compressIds(r.found) }))}</b></p><table><thead><tr><th>ID</th><th>${esc(t('col_model'))}</th><th>${esc(t('col_supply'))}</th><th>${esc(t('col_prot'))}</th></tr></thead><tbody>${rows.join('')}</tbody></table>` : `<p><b>${esc(t('none_found'))}</b></p>`;
    if (r.garbled.length) html += `<p style="color:#c62828"><b>${esc(t('duplicates', { ids: compressIds(r.garbled) }))}</b></p>`;
    if (missing.length && r.found.length) html += `<p style="color:#c77700">${esc(t('missing', { ids: compressIds(missing) }))}</p>`;
    $('chain').innerHTML = html;
    const ok = r.found.length && !r.garbled.length && !missing.length;
    status(ok ? 'finished' : 'warning', ok ? '✓' : '!', ok ? t('chain_ok', { n: r.found.length }) : t('chain_check_list'), '');
    log(`scan: ${JSON.stringify(r)}`);
  });

  // ---------------- center ----------------
  $('center').onclick = () => exclusive(async () => {
    status('working', '…', t('center_search'), '');
    const f = await bus.findSingle();
    if (!f) { status('error', '!', t('center_none'), ''); return; }
    const s = await bus.identify(f.id);
    if (s.family !== 'sts') { status('warning', '!', t('center_unsupported', { model: s.model }), ''); return; }
    status('working', String(f.id), t('centering', { id: f.id }), servoLine(s));
    try {
      held = f.id; refresh();
      const p = await bus.center(f.id, (pos) => { $('det').textContent = t('position', { p: pos }); });
      status('success', String(f.id), t('centered', { id: f.id }), t('position', { p }));
      log(`centered ID ${f.id} at ${p}`);
    } catch (e) {
      held = null;
      if (e.kind === 'failed' && e.message.startsWith('power:')) status('error', '!', t('center_power', { v: v1(e.message.slice(6)) }), t('center_power_det'));
      else throw e;
    }
  });
  $('release').onclick = () => exclusive(async () => { if (held != null) { await bus.release(held); log('released ' + held); } held = null; status('', 'ID', t('released'), ''); });
  window.addEventListener('beforeunload', () => { if (held != null && bus) bus.release(held); });

  status('', 'ID', supported ? t('start_here') : t('unsupported_short'), supported ? t('start_here_det') : '');
  refresh();
})();
