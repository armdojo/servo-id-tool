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
