#!/usr/bin/env python3
"""The CAN bus in a browser, live.

    canweb.py [--iface can0] [--port 8091]
    then open http://localhost:8091/

Written because the bus was refusing to talk and the question had stopped being
"what do the frames say" and become "is anything arriving at all". Those are
different questions and the terminal answers them badly: candump prints nothing
for both "no traffic" and "traffic that cannot be decoded", and the error counters
that tell them apart live somewhere else entirely.

So the page shows three things at once:

  the electrical state   frames rising            -> decoded, working
                         err-warn/err-pass rising -> edges arrive, undecodable
                                                     (wrong bitrate, or one wire)
                         everything flat at zero  -> no edges at all: nothing is
                                                     driving the line, or H and L
                                                     are swapped, which makes
                                                     every dominant bit read as
                                                     recessive and is silent
  every id seen          with its AgileX meaning and how often it arrives
  the last frames        raw, newest first

The distinction in the middle matters most. A swapped pair and a dead bus look
identical from userspace, and the only way to tell is that a swapped pair usually
still produces *some* error frames when the vehicle transmits, while a dead bus
produces literally nothing.
"""
import argparse
import json
import re
import subprocess
import threading
import time
from collections import deque, defaultdict
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# What the ids mean, from doc/CAN.md and the AgileX manual. Both generations are
# listed because which one a chassis speaks is not knowable until it speaks: the
# same id means different things across them, so the label says so rather than
# guessing.
# The ids, from doc/CAN.md, which was written from the AgileX protocol document
# rather than from memory. The first version of this table was from memory and got
# several of them wrong - 0x261 is the BMS in that version and the actuator low-speed
# block here - which is exactly the kind of error that makes a decoder look like it
# is working while it labels the wrong thing.
IDS = {
    # feedback, v2 (what a SCOUT MINI OMNI sends)
    0x211: "system state: vehicle state, control mode, battery 0.1V, errors",
    0x221: "motion state: linear, angular, lateral, steering (mm/s, mrad/s)",
    0x231: "light state",
    0x241: "RC state",
    0x251: "actuator 1 hi: rpm, current 0.1A, pulses",
    0x252: "actuator 2 hi", 0x253: "actuator 3 hi", 0x254: "actuator 4 hi",
    0x255: "actuator 5 hi", 0x256: "actuator 6 hi",
    0x257: "actuator 7 hi", 0x258: "actuator 8 hi",
    0x261: "actuator 1 lo: driver V, driver temp, motor temp, state",
    0x262: "actuator 2 lo", 0x263: "actuator 3 lo", 0x264: "actuator 4 lo",
    0x265: "actuator 5 lo", 0x266: "actuator 6 lo",
    0x267: "actuator 7 lo", 0x268: "actuator 8 lo",
    0x291: "motion mode state",
    0x311: "odometry: left and right wheel, int32",
    0x361: "BMS: SoC, SoH, volts, amps, temperature (0.1 units)",
    # commands - seen only if something else is driving the vehicle
    0x111: "v2 motion command (mm/s, big-endian)",
    0x121: "v2 light command",
    0x131: "v1 motion state / v2 command",
    0x141: "v1 command",
    0x130: "v1 motion command (percent, checksum, rolling count)",
    0x151: "v1 system state",
}

state = {
    "iface": "can0",
    "frames": deque(maxlen=200),
    "ids": defaultdict(lambda: {"count": 0, "last": "", "at": 0.0}),
    "counters": {},
    "total": 0,
    "started": time.time(),
}
lock = threading.Lock()


def adapter_probe(dev):
    """Ask the adapter who it is, which also proves writes reach it.

    slcan's `V` returns a version string. Getting one back is the only cheap proof
    that the host-to-adapter direction works at all - and that mattered here,
    because a silent CAN bus and a dead write path look identical from every other
    angle. The CANable2 answered
    `16e7497-dirty github.com/normaldotcom/canable2.git`, so the silence is beyond
    the adapter rather than inside it.

    Only run when the bus is not up: slcand owns the port while it is, and two
    readers on one serial line is how a stream turns to nonsense.
    """
    try:
        import serial
    except Exception:
        return None
    try:
        with serial.Serial(dev, 115200, timeout=0.6) as s:
            s.write(b"C\r")
            time.sleep(0.15)
            s.reset_input_buffer()
            s.write(b"V\r")
            time.sleep(0.3)
            v = s.read(96).decode(errors="replace").strip()
            return v or None
    except Exception:
        return None


def read_counters(iface):
    """RX/TX and the CAN error states, from `ip -s -details link`.

    Parsed rather than read from sysfs because slcan does not publish the same
    files a native controller does, and this has to work for both.
    """
    try:
        out = subprocess.run(["ip", "-s", "-details", "link", "show", iface],
                             capture_output=True, text=True, timeout=3).stdout
    except Exception:
        return {"up": False}
    c = {"up": "UP" in out.split("\n")[0]}
    m = re.search(r"can state (\S+)", out)
    c["state"] = m.group(1) if m else "?"
    m = re.search(
        r"re-started bus-errors arbit-lost error-warn error-pass bus-off\s*\n\s*"
        r"(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)", out)
    if m:
        for k, v in zip(("restarts", "bus_errors", "arbit_lost",
                         "err_warn", "err_pass", "bus_off"), m.groups()):
            c[k] = int(v)
    rx = re.search(r"RX:.*\n\s*(\d+)\s+(\d+)\s+(\d+)", out)
    tx = re.search(r"TX:.*\n\s*(\d+)\s+(\d+)\s+(\d+)", out)
    if rx:
        c["rx_bytes"], c["rx_packets"], c["rx_errors"] = (int(x) for x in rx.groups())
    if tx:
        c["tx_bytes"], c["tx_packets"], c["tx_errors"] = (int(x) for x in tx.groups())
    return c


def dumper(iface):
    """candump, parsed. Errors included, because they are the interesting case."""
    while True:
        p = subprocess.Popen(["candump", "-e", "-t", "a", f"{iface},0:0,#FFFFFFFF"],
                             stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                             text=True, bufsize=1)
        for line in p.stdout:
            parts = line.split()
            if len(parts) < 3:
                continue
            try:
                ts = float(parts[0].strip("()"))
            except ValueError:
                ts = time.time()
            cid = parts[2]
            data = " ".join(parts[3:])
            with lock:
                state["total"] += 1
                state["frames"].appendleft(
                    {"t": ts, "id": cid, "data": data})
                e = state["ids"][cid]
                e["count"] += 1
                e["last"] = data
                e["at"] = ts
        p.wait()
        time.sleep(1)


def poller(iface):
    while True:
        c = read_counters(iface)
        with lock:
            state["counters"] = c
        time.sleep(0.5)


PAGE = r"""<!doctype html>
<meta charset="utf-8"><title>CAN</title>
<style>
 :root{--bg:#f2f2f5;--card:#fff;--ink:#1c1c1e;--dim:#8a8a8e;--good:#34c759;--bad:#ff3b30;--warn:#c77700}
 *{box-sizing:border-box} body{margin:0;padding:14px;background:var(--bg);color:var(--ink);
   font:14px/1.45 -apple-system,"Segoe UI",Roboto,system-ui,sans-serif;font-feature-settings:"tnum"}
 h2{margin:0 0 8px;font-size:12px;letter-spacing:.07em;text-transform:uppercase}
 .row{display:grid;gap:12px;grid-template-columns:1fr 1fr 1.2fr}
 .card{background:var(--card);border-radius:12px;padding:12px 14px;box-shadow:0 1px 3px rgba(0,0,0,.1)}
 .big{font-size:30px;font-weight:600;line-height:1.1}
 .k{display:flex;justify-content:space-between;padding:3px 0;border-bottom:1px solid #eee;font-size:13px}
 .k:last-child{border:0} .dim{color:var(--dim)} .good{color:var(--good)} .bad{color:var(--bad)} .warn{color:var(--warn)}
 table{width:100%;border-collapse:collapse;font-size:13px} td,th{text-align:left;padding:3px 6px}
 th{color:var(--dim);font-weight:500;font-size:11px;text-transform:uppercase}
 tr:nth-child(even){background:#fafafc}
 code{font-family:ui-monospace,Menlo,monospace}
 #verdict{margin:12px 0;padding:12px 14px;border-radius:12px;background:#fff;
   border-left:4px solid var(--dim);box-shadow:0 1px 3px rgba(0,0,0,.1)}
</style>
<div class="row">
  <div class="card"><h2>Frames</h2><div class="big" id="total">0</div>
    <div class="dim" id="rate">0 /s</div></div>
  <div class="card"><h2>Bus</h2><div id="bus"></div></div>
  <div class="card"><h2>Errors</h2><div id="err"></div>
    <div class="dim" id="adapter" style="margin-top:6px;font-size:11px"></div></div>
</div>
<div id="verdict">waiting…</div>
<div class="row" style="grid-template-columns:1fr 1fr">
  <div class="card"><h2>Ids seen</h2><table id="ids"><tr><td class="dim">none yet</td></tr></table></div>
  <div class="card"><h2>Last frames</h2><table id="last"><tr><td class="dim">none yet</td></tr></table></div>
</div>
<script>
let prev=0, prevT=Date.now();
async function tick(){
  const r = await fetch('/data'); const d = await r.json();
  document.getElementById('total').textContent = d.total;
  const now=Date.now(); const dt=(now-prevT)/1000;
  if(dt>0.4){ document.getElementById('rate').textContent =
      ((d.total-prev)/dt).toFixed(1)+' /s'; prev=d.total; prevT=now; }
  const c=d.counters||{};
  const kv=(k,v,cls)=>`<div class="k"><span class="dim">${k}</span><span class="${cls||''}">${v}</span></div>`;
  document.getElementById('bus').innerHTML =
    kv('link', c.up?'up':'down', c.up?'good':'bad') +
    kv('state', c.state||'?', c.state==='ERROR-ACTIVE'?'good':(c.state?'warn':'')) +
    kv('rx packets', c.rx_packets ?? '-', c.rx_packets>0?'good':'dim') +
    kv('tx packets', c.tx_packets ?? '-');
  document.getElementById('err').innerHTML =
    kv('bus errors', c.bus_errors ?? '-', c.bus_errors>0?'bad':'dim') +
    kv('error warn', c.err_warn ?? '-', c.err_warn>0?'warn':'dim') +
    kv('error passive', c.err_pass ?? '-', c.err_pass>0?'bad':'dim') +
    kv('bus off', c.bus_off ?? '-', c.bus_off>0?'bad':'dim');
  // The three-way reading this page exists for.
  let v='', border='var(--dim)';
  if(d.total>0){ v='<b>Traffic is arriving and decoding.</b> The link is good — read the ids below.'; border='var(--good)'; }
  else if((c.bus_errors||0)+(c.err_warn||0)+(c.err_pass||0)>0){
    v='<b>Edges arrive but cannot be decoded.</b> Something is driving the line, so the pair is connected — but the bits do not parse. Wrong bitrate, or only one of CAN_H/CAN_L is landing.'; border='var(--warn)'; }
  else if(c.up){ v='<b>Nothing on the line at all.</b> No frames and no errors: either nothing is transmitting, or CAN_H and CAN_L are swapped — a swapped pair reads every dominant bit as recessive and is completely silent.'; border='var(--bad)'; }
  else { v='Interface is down.'; border='var(--bad)'; }
  const el=document.getElementById('verdict'); el.innerHTML=v; el.style.borderLeftColor=border;
  document.getElementById('adapter').textContent = d.adapter ? ('adapter: '+d.adapter) : '';

  let h='<tr><th>id</th><th>meaning</th><th>n</th><th>last data</th></tr>';
  for(const [id,e] of Object.entries(d.ids).sort((a,b)=>b[1].count-a[1].count))
    h+=`<tr><td><code>${id}</code></td><td class="dim">${e.meaning||''}</td><td>${e.count}</td><td><code>${e.last}</code></td></tr>`;
  document.getElementById('ids').innerHTML = Object.keys(d.ids).length? h : '<tr><td class="dim">none yet</td></tr>';

  let f='<tr><th>t</th><th>id</th><th>data</th></tr>';
  for(const x of d.frames.slice(0,18))
    f+=`<tr><td class="dim">${x.t.toFixed(3)}</td><td><code>${x.id}</code></td><td><code>${x.data}</code></td></tr>`;
  document.getElementById('last').innerHTML = d.frames.length? f : '<tr><td class="dim">none yet</td></tr>';
}
setInterval(tick, 400); tick();
</script>
"""


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path.startswith("/data"):
            with lock:
                ids = {k: {"count": v["count"], "last": v["last"],
                           "meaning": IDS.get(int(k, 16), "") if
                           re.fullmatch(r"[0-9A-Fa-f]+", k) else ""}
                       for k, v in state["ids"].items()}
                body = json.dumps({
                    "total": state["total"],
                    "counters": state["counters"],
                    "ids": ids,
                    "frames": list(state["frames"])[:40],
                    "adapter": state.get("adapter"),
                }).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        body = PAGE.encode()
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="can0")
    ap.add_argument("--port", type=int, default=8091)
    ap.add_argument("--bind", default="127.0.0.1")
    ap.add_argument("--dev", default="",
                    help="the adapter's serial device, to identify it at startup")
    a = ap.parse_args()
    state["iface"] = a.iface
    # Probed once at startup, before slcand can be holding the port.
    state["adapter"] = adapter_probe(a.dev) if a.dev else None
    threading.Thread(target=dumper, args=(a.iface,), daemon=True).start()
    threading.Thread(target=poller, args=(a.iface,), daemon=True).start()
    print(f"  {a.iface} -> http://localhost:{a.port}/")
    ThreadingHTTPServer((a.bind, a.port), H).serve_forever()


if __name__ == "__main__":
    main()
