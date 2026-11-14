/* ArmDojo Servo ID tool — Web Serial link + page UI. Mounts into <div id="armdojo-servo-tool">.
 * Language: data-lang on the container, else <html lang>, else English. Demo (no hardware): add ?servo-demo=1 to the URL. */
(function () {
  'use strict';
  const C = window.ArmDojoServoCore, M = window.ArmDojoServoMock, I18N = window.ArmDojoServoI18n || {};
  const { Bus, Numberer, BusError, compressIds, sleep, now } = C;

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
    <span class="muted">${esc(t('presets'))}</span><button class="pre" data-p="1-6">1–6</button><button class="pre" data-p="1-12">1–12</button><button class="pre" data-p="1-15">1–15</button><button class="pre" data-p="1-18">1–18</button></div>
  <div class="row" style="margin-top:8px"><label><input type="checkbox" checked data-r="force1m"> ${esc(t('force_1m'))}</label>
    <label><input type="checkbox" checked data-r="speak"> ${esc(t('speak'))}</label></div>
  <div class="note">${esc(t('one_at_a_time'))}</div>
  <div class="row"><button data-r="start" disabled>${esc(t('btn_start'))}</button><button class="sec" data-r="stop" hidden>${esc(t('btn_stop'))}</button></div>
  <table data-r="table" hidden><thead><tr><th>${esc(t('col_new'))}</th><th>${esc(t('col_was'))}</th><th>${esc(t('col_model'))}</th><th>${esc(t('col_supply'))}</th></tr></thead><tbody data-r="rows"></tbody></table></div>
<div class="card"><h3>3. ${esc(t('step_check'))}</h3><p class="muted" style="margin:0 0 8px">${esc(t('check_hint'))}</p>
  <div class="row"><button data-r="check" disabled>${esc(t('btn_check'))}</button></div><div data-r="chain" style="margin-top:8px"></div></div>
<div class="card"><h3>${esc(t('step_center'))}</h3><p class="muted" style="margin:0 0 8px">${esc(t('center_hint'))}</p>
  <div class="row"><button data-r="center" disabled>${esc(t('btn_center'))}</button><button class="sec" data-r="release" hidden>${esc(t('btn_release'))}</button></div></div>
<details><summary>${esc(t('log'))}</summary><pre data-r="log"></pre></details>`;
  const $ = (r) => root.querySelector(`[data-r="${r}"]`);

  // ---------------- state ----------------
  let link = null, bus = null, numberer = null, busy = false, held = null, lastNumbered = null;
  const done = new Set();

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
  root.querySelectorAll('[data-p]').forEach((b) => { b.onclick = () => { const [a, z] = b.dataset.p.split('-'); $('from').value = a; $('to').value = z; }; });

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
    if (!(from >= 0 && to <= 253 && from <= to)) { status('error', '!', t('bad_range'), ''); return; }
    if (held) { try { await bus.release(held); } catch (_) {} held = null; }
    numberer = new Numberer(bus, { next: from, end: to, force1M: $('force1m').checked, done, lastNumbered }, (s) => {
      const [m, d] = (MSG[s.key] || (() => [s.key, '']))(s.args || {});
      const extra = s.servo ? servoLine(s.servo) : '';
      status(s.phase, s.big, m, [d, extra].filter(Boolean).join(' · '));
      if (s.key === 'io_lost') lost();
    }, (old, v, next) => {
      lastNumbered = { id: v.id, baud: v.baud };
      $('table').hidden = false;
      const tr = document.createElement('tr');
      tr.innerHTML = `<td><b>${v.id}</b></td><td>${old}</td><td>${esc(v.model)}</td><td>${v.voltage != null ? v1(v.voltage) + ' V' : ''}</td>`;
      $('rows').prepend(tr);
      $('from').value = next;
      say(v.id); log(`ID ${old} → ${v.id} (${v.model}, ${v.voltage} V)`);
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
    for (const id of r.found) { let s = null; try { s = await bus.identify(id); } catch (_) {} rows.push(`<tr><td><b>${id}</b></td><td>${esc(s ? s.model : '?')}</td><td>${s && s.voltage != null ? v1(s.voltage) + ' V' : ''}</td></tr>`); }
    const from = parseInt($('from').value, 10) || 1, to = parseInt($('to').value, 10) || 0;
    const lo = Math.min(from, ...(r.found.length ? r.found : [from])), hi = Math.max(to, ...(r.found.length ? r.found : [to]));
    const missing = []; for (let i = Math.max(lo, 1); i <= hi; i++) if (!r.found.includes(i) && !r.garbled.includes(i)) missing.push(i);
    let html = r.found.length ? `<p><b>${esc(t('found', { n: r.found.length, ids: compressIds(r.found) }))}</b></p><table><thead><tr><th>ID</th><th>${esc(t('col_model'))}</th><th>${esc(t('col_supply'))}</th></tr></thead><tbody>${rows.join('')}</tbody></table>` : `<p><b>${esc(t('none_found'))}</b></p>`;
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
