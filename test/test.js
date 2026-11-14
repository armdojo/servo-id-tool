// node servo-id/web/test/test.js — protocol + numbering checks against simulated servos (same coverage as the Kotlin SelfTest).
'use strict';
const core = require('../src/core.js');
const { MockServo, MockLink, DemoBench } = require('../src/mock.js');
const { Bus, Numberer, hex, packet } = core;

let pass = 0; const fail = [];
const check = (name, ok, detail) => { if (ok) pass++; else fail.push(name); console.log((ok ? 'PASS ' : 'FAIL ') + name + (detail ? ': ' + detail : '')); };
const throws = async (kind, f) => { try { await f(); return false; } catch (e) { return e.kind === kind; } };
const bus = (opt, ...ids) => { const m = new MockLink(opt); ids.forEach((i) => m.add(new MockServo(i))); return [new Bus(m), m]; };

(async () => {
  check('PING packet', hex(packet(1, 1)) === 'FF FF 01 02 01 FB');
  check('write-ID packet', hex(packet(1, 3, [5, 7])) === 'FF FF 01 04 03 05 07 EB', hex(packet(1, 3, [5, 7])));

  { const [b, m] = bus({}, 1);
    check('ping finds ID 1', await b.ping(1));
    check('ping on a missing ID returns false', !(await b.ping(2, 5)));
    m.servo(1).mem[62] = 118;
    check('read register', (await b.read(1, 62, 1))[0] === 118);
    await b.write(1, 48, [44, 1]);
    check('write then read back', (await b.read16(1, 48)) === 300);
    check('identify STS3215 12 V', (await b.identify(1)).model === 'STS3215 (12 V)');
    check('no reply → noreply error', await throws('noreply', () => b.read(9, 0, 1))); }

  { const [b] = bus({ echo: true }, 1);
    check('board that echoes TX', (await b.ping(1)) && (await b.read(1, 14, 1))[0] === 140); }

  { const [b, m] = bus({}, 1); m.servo(1).noise = [0x00, 0x13, 0xFF];
    check('resync after noise', await b.ping(1)); }

  { const [b, m] = bus({}, 3); m.add(new MockServo(3));
    check('two servos with the same ID → collision', await throws('collision', () => b.ping(3))); }

  { const [b, m] = bus({}, 4); m.servo(4).replyAs = 5;
    check('wrong ID answers → collision', await throws('collision', () => b.ping(4)));
    m.servo(4).replyAs = null; m.servo(4).corrupt = true;
    check('bad checksum → collision', await throws('collision', () => b.ping(4))); }

  { const m = new MockLink(); m.add(new MockServo(7, { baud: 115200 })); const b = new Bus(m);
    const f = await b.findSingle();
    check('broadcast search over every baud', f && f.id === 7 && f.baud === 115200, JSON.stringify(f)); }

  { const [b, m] = bus({}, 1, 3, 7); m.add(new MockServo(9)); m.add(new MockServo(9));
    const s = await b.scan(null, 2);
    check('chain scan', JSON.stringify(s.found) === '[1,3,7]' && JSON.stringify(s.garbled) === '[9]', JSON.stringify(s)); }

  { const [b, m] = bus({}, 1);
    const s = await b.identify(1);
    await b.changeId(s, 5);
    check('change ID 1 → 5 and lock', s.id === 5 && m.servos[0].id === 5 && m.servos[0].mem[55] === 1);
    check('old ID silent', !(await b.ping(1, 5)));
    m.add(new MockServo(6));
    check('target ID taken → refused', await throws('failed', () => b.changeId(s, 6)));
    await b.setBaudRegister(s, 500000);
    check('baud register to 500k', m.servos[0].baud === 500000 && b.baud === 500000 && await b.ping(5)); }

  { const [b, m] = bus({}, 1); m.servo(1).pos = 1500;
    const p = await b.center(1);
    check('center: 1500 → 2048, holding at 30 %', Math.abs(p - 2048) <= 8 && m.servo(1).torque && m.servo(1).w(48) === 300, String(p));
    await b.release(1); check('release torque', !m.servo(1).torque);
    m.servo(1).mem[62] = 50;
    check('center refuses on USB power only', await throws('failed', () => b.center(1))); }

  // One-at-a-time numbering with the demo bench: servos at mixed bauds appear one by one.
  { const m = new DemoBench(); const b = new Bus(m);
    const got = []; const statuses = [];
    const n = new Numberer(b, { next: 1, end: 3, force1M: true }, (s) => statuses.push(s.key), (old, v) => got.push([old, v.id, v.baud]));
    const t0 = Date.now(); await n.run();
    check('numbering 1–3 with plug/unplug (incl. 115200 and 500k servos)', JSON.stringify(got.map((g) => g[1])) === '[1,2,3]' && got.every((g) => g[2] === 1000000),
      JSON.stringify(got) + ` in ${((Date.now() - t0) / 1000).toFixed(1)} s`);
    check('status flow', statuses.includes('plug_next') && statuses.includes('writing') && statuses.includes('done') && statuses[statuses.length - 1] === 'finished', statuses.join(',')); }

  { const m = new MockLink(); m.add(new MockServo(1)); m.add(new MockServo(1)); const b = new Bus(m);
    const keys = [];
    const n = new Numberer(b, { next: 1, end: 2 }, (s) => { keys.push(s.key); if (s.key === 'collision') n.stop(); });
    await n.run();
    check('two servos plugged at once → asks for one', keys.includes('collision'), keys.join(',')); }

  console.log(fail.length ? `FAILED ${fail.length}: ${fail.join('; ')}` : `ALL PASS (${pass})`);
  process.exit(fail.length ? 1 : 0);
})();
