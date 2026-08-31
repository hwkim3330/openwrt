#!/usr/bin/env python3
"""The vehicle's RS232 telemetry in a browser.

    serweb.py [--dev /dev/ttyUSB0] [--baud 115200] [--port 8092]
    then open http://localhost:8092/

The SCOUT MINI ships a USB-to-RS232 lead alongside the USB-to-CAN one, and the
chassis talks on it continuously. That mattered here for a reason beyond telemetry:
CAN had gone completely silent and the working conclusion was becoming "the vehicle
is broken". It is not - this port is alive and framing perfectly, which puts the
fault between the CAN connector and the adapter rather than in the chassis.

The framing was found by sweeping baud rates and looking at the bytes rather than
by being told. At 115200 the stream is:

    5A A5 0A AA 06 00 00 00 00 1C 00 4C 21
    5A A5 0A AA 07 00 00 00 02 0C 00 27 EF
    ^^^^^ ^^ ^^ ^^                   ^^^^^
    sync  len ?  seq                 checksum?

13 bytes, a 5A A5 sync word, a length byte of 0x0A, and a counter in byte 4 that
increments 01, 02, 03 ... So the frames are decoded structurally - sync, length,
sequence, payload, tail - and the payload is shown raw. Naming the payload fields
would be guessing: this is not the CAN protocol and no document for it has been
found, so the page shows what is there and does not pretend to know more.
"""
import argparse
import json
import threading
import time
from collections import deque, defaultdict
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

SYNC = b"\x5a\xa5"

state = {
    "frames": deque(maxlen=300),
    "bytes": 0,
    "total": 0,
    "bad": 0,
    "seq": defaultdict(int),
    "started": time.time(),
    "dev": "", "baud": 0,
}
lock = threading.Lock()


def reader(dev, baud):
    import serial
    while True:
        try:
            s = serial.Serial(dev, baud, timeout=1)
        except Exception as e:
            with lock:
                state["error"] = f"{type(e).__name__}: {e}"
            time.sleep(2)
            continue
        with lock:
            state.pop("error", None)
        buf = bytearray()
        while True:
            try:
                chunk = s.read(256)
            except Exception:
                break
            if not chunk:
                continue
            buf += chunk
            with lock:
                state["bytes"] += len(chunk)
            # Resynchronise on the sync word rather than assuming alignment: the
            # stream is joined mid-frame whenever this starts.
            while True:
                i = buf.find(SYNC)
                if i < 0:
                    if len(buf) > 4096:
                        del buf[:-2]
                    break
                if i:
                    with lock:
                        state["bad"] += i
                    del buf[:i]
                if len(buf) < 4:
                    break
                ln = buf[2]
                total = ln + 3          # sync(2) + len(1) + ln bytes
                if len(buf) < total:
                    break
                f = bytes(buf[:total])
                del buf[:total]
                with lock:
                    state["total"] += 1
                    seq = f[4] if len(f) > 4 else -1
                    state["seq"][seq] += 1
                    state["frames"].appendleft({
                        "t": time.time(),
                        "hex": f.hex(" "),
                        "len": ln,
                        "seq": seq,
                        "payload": f[5:-2].hex(" ") if len(f) > 7 else "",
                        "tail": f[-2:].hex(" "),
                    })
        try:
            s.close()
        except Exception:
            pass


PAGE = r"""<!doctype html>
<meta charset="utf-8"><title>vehicle serial</title>
<style>
 :root{--bg:#f2f2f5;--card:#fff;--ink:#1c1c1e;--dim:#8a8a8e;--good:#34c759;--bad:#ff3b30}
 *{box-sizing:border-box}body{margin:0;padding:14px;background:var(--bg);color:var(--ink);
  font:14px/1.45 -apple-system,"Segoe UI",Roboto,system-ui,sans-serif;font-feature-settings:"tnum"}
 h2{margin:0 0 8px;font-size:12px;letter-spacing:.07em;text-transform:uppercase}
 .row{display:grid;gap:12px;grid-template-columns:repeat(4,1fr)}
 .card{background:var(--card);border-radius:12px;padding:12px 14px;box-shadow:0 1px 3px rgba(0,0,0,.1)}
 .big{font-size:28px;font-weight:600;line-height:1.1}.dim{color:var(--dim)}
 .good{color:var(--good)}.bad{color:var(--bad)}
 table{width:100%;border-collapse:collapse;font-size:12.5px}td,th{text-align:left;padding:3px 6px}
 th{color:var(--dim);font-weight:500;font-size:11px;text-transform:uppercase}
 tr:nth-child(even){background:#fafafc}code{font-family:ui-monospace,Menlo,monospace}
 .wide{grid-column:1/-1}
</style>
<div class="row">
  <div class="card"><h2>Frames</h2><div class="big" id="total">0</div><div class="dim" id="rate">0 /s</div></div>
  <div class="card"><h2>Bytes</h2><div class="big" id="bytes">0</div><div class="dim" id="dev"></div></div>
  <div class="card"><h2>Unframed</h2><div class="big" id="bad">0</div><div class="dim">bytes outside a frame</div></div>
  <div class="card"><h2>Verdict</h2><div id="verdict" class="dim">…</div></div>
</div>
<div class="row" style="margin-top:12px;grid-template-columns:1fr 2fr">
  <div class="card"><h2>Sequence byte</h2><table id="seq"></table></div>
  <div class="card"><h2>Last frames</h2><table id="last"></table></div>
</div>
<script>
let prev=0,prevT=Date.now();
async function tick(){
 const d=await (await fetch('/data')).json();
 total.textContent=d.total; bytes.textContent=d.bytes; bad.textContent=d.bad;
 dev.textContent=d.dev+' @ '+d.baud;
 const now=Date.now(),dt=(now-prevT)/1000;
 if(dt>0.4){rate.textContent=((d.total-prev)/dt).toFixed(1)+' /s';prev=d.total;prevT=now;}
 verdict.innerHTML = d.error ? '<span class="bad">'+d.error+'</span>'
   : d.total>0 ? '<span class="good">framing cleanly — the chassis is alive and transmitting</span>'
   : '<span class="bad">no frames</span>';
 let s='<tr><th>byte 4</th><th>count</th></tr>';
 for(const [k,v] of Object.entries(d.seq).sort((a,b)=>a[0]-b[0])) s+=`<tr><td><code>${k}</code></td><td>${v}</td></tr>`;
 seq.innerHTML=s;
 let f='<tr><th>seq</th><th>len</th><th>payload</th><th>tail</th></tr>';
 for(const x of d.frames.slice(0,20))
   f+=`<tr><td><code>${x.seq}</code></td><td>${x.len}</td><td><code>${x.payload}</code></td><td class="dim"><code>${x.tail}</code></td></tr>`;
 last.innerHTML=f;
}
setInterval(tick,400);tick();
</script>
"""


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path.startswith("/data"):
            with lock:
                body = json.dumps({
                    "total": state["total"], "bytes": state["bytes"],
                    "bad": state["bad"], "dev": state["dev"], "baud": state["baud"],
                    "seq": dict(state["seq"]),
                    "frames": list(state["frames"])[:40],
                    **({"error": state["error"]} if "error" in state else {}),
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
    ap.add_argument("--dev", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--port", type=int, default=8092)
    ap.add_argument("--bind", default="127.0.0.1")
    a = ap.parse_args()
    state["dev"], state["baud"] = a.dev, a.baud
    threading.Thread(target=reader, args=(a.dev, a.baud), daemon=True).start()
    print(f"  {a.dev} @ {a.baud} -> http://localhost:{a.port}/")
    ThreadingHTTPServer((a.bind, a.port), H).serve_forever()


if __name__ == "__main__":
    main()
