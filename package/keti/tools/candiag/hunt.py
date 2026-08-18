#!/usr/bin/env python3
"""Try every way of listening until something decodes.

The receive path is proven (a local echo round-trips), the adapter is healthy,
and the vehicle is powered - so if nothing decodes, the setting is wrong or the
signal is not really there. This walks the settings so that conclusion can be
reached by exhaustion rather than by assumption.
"""
import os, re, socket, struct, subprocess, sys, time

RATES = [1000000, 800000, 666000, 500000, 400000, 250000, 200000, 125000,
         100000, 83333, 62500, 50000, 33333, 20000, 10000]
SP_ = [None, 0.75]
FD = [(500000, 2000000), (500000, 5000000), (1000000, 4000000), (250000, 1000000)]
DWELL = 2.5
LABEL = {0x151:"v1 SYSTEM (=> v1)",0x131:"v1 motion / v2 brake",0x130:"v1 cmd",
         0x211:"v2 system",0x221:"v2 MOTION (=> v2)",0x241:"v2 RC (=> v2)",
         0x111:"v2 cmd",0x311:"v2 odom",0x361:"v2 BMS"}

def sh(*a): return subprocess.run(a,capture_output=True,text=True).stdout
def up(iface,args):
    sh("ip","link","set",iface,"down")
    r=subprocess.run(["ip","link","set",iface,"up","type","can"]+args,
                     capture_output=True,text=True)
    return r.returncode==0
def berr(i):
    m=re.search(r"berr-counter tx (\d+) rx (\d+)",sh("ip","-d","link","show",i))
    return m.groups() if m else ("?","?")

def listen(iface,secs,fd):
    s=socket.socket(socket.PF_CAN,socket.SOCK_RAW,1)
    s.setsockopt(socket.SOL_CAN_RAW,2,struct.pack("=I",0x1FFFFFFF))
    if fd:
        try: s.setsockopt(socket.SOL_CAN_RAW,5,struct.pack("=i",1))
        except OSError: pass
    s.bind((iface,)); s.settimeout(0.2)
    f={}; e=0; t0=time.time()
    while time.time()-t0<secs:
        try: raw=s.recv(72)
        except socket.timeout: continue
        except OSError: break
        if len(raw)<8: continue
        cid,dlc=struct.unpack("=IB3x",raw[:8])
        if cid & 0x20000000: e+=1; continue
        k=cid & (0x1FFFFFFF if cid & 0x80000000 else 0x7FF)
        d=f.setdefault(k,{"n":0,"last":b""}); d["n"]+=1; d["last"]=raw[8:8+min(dlc,64)]
    s.close(); return f,e

combos=[]
for r in RATES:
    for sp in SP_:
        a=["bitrate",str(r),"restart-ms","100"]
        if sp: a+=["sample-point",str(sp)]
        combos.append((f"{r} sp={sp or 'default'}",a,False))
        combos.append((f"{r} sp={sp or 'default'} listen-only",a+["listen-only","on"],False))
for r,dr in FD:
    combos.append((f"FD {r}/{dr}",["bitrate",str(r),"dbitrate",str(dr),"fd","on","restart-ms","100"],True))

iface=sys.argv[1] if len(sys.argv)>1 else "can0"
print(f"{len(combos)} settings on {iface}, {DWELL}s each "
      f"(~{len(combos)*DWELL/60:.1f} min per pass)\n",flush=True)
for p in range(1,6):
    hits=[]
    for how,args,fd in combos:
        if not up(iface,args):
            continue
        f,e=listen(iface,DWELL,fd)
        n=sum(v["n"] for v in f.values()); tx,rx=berr(iface)
        if n:
            print(f"\n  *** {how}: {n} FRAMES ***",flush=True)
            for k in sorted(f):
                print(f"      {k:03X} n={f[k]['n']:<5d} {f[k]['last'].hex(' ')}"
                      f"   {LABEL.get(k,'')}",flush=True)
            sys.exit(0)
        if e or rx!="0":
            hits.append(f"{how} (errfr={e} rx={rx})")
    print(f"  pass {p}: no frames. settings that produced errors: "
          f"{len(hits)}",flush=True)
    for h in hits[:6]: print(f"      {h}",flush=True)
print("\nexhausted - nothing decodes at any setting")
