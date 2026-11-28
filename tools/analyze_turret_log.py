#!/usr/bin/env python3 -I
"""Summarise a BIOBUZZ flight-recorder run. Usage: analyze_turret_log.py <run_dir_or_loop.csv>"""
import csv, glob, math, os, statistics as st, sys

def load(path):
    if os.path.isdir(path):
        files = sorted(glob.glob(os.path.join(path, "*_loop.csv")))
        if not files: sys.exit("no *_loop.csv in " + path)
        path = files[-1]
    rows = [r for r in csv.DictReader(open(path))
            if None not in r.values() and None not in r]  # drop a truncated last row
    data = {k: [float(r[k]) if r[k] != "" else math.nan for r in rows] for k in rows[0]}
    return path, data

def pct(xs, p):
    xs = sorted(x for x in xs if not math.isnan(x))
    return xs[min(len(xs) - 1, int(len(xs) * p))] if xs else math.nan

def main():
    path, d = load(sys.argv[1] if len(sys.argv) > 1 else ".")
    n = len(d["t_ms"]); t = d["t_ms"]
    print(f"FILE {path}\nrows={n} duration={(t[-1]-t[0])/1000:.1f}s")
    dt = d.get("op_dt_ms", [])
    print(f"\n== LOOP ==  dt ms  p50={pct(dt,.5):.1f} p95={pct(dt,.95):.1f} max={max(dt):.1f}  (rate≈{1000/pct(dt,.5):.0f} Hz)")
    for k in ("tur_upd_us","tur_odo_us","tur_vision_us","tur_ctrl_us"):
        if k in d: print(f"   {k:14s} p50={pct(d[k],.5):7.0f} p95={pct(d[k],.95):7.0f}")

    if "vis_new_frame" in d:
        fr = d["vis_frame_interval_ms"]; vis = d["vis_visible"]
        print(f"\n== VISION ==  visible {100*sum(vis)/n:.0f}% of loops, new frames={sum(d['vis_new_frame']):.0f}")
        print(f"   frame interval ms p50={pct(fr,.5):.1f} p95={pct(fr,.95):.1f}")
        print(f"   total latency ms  p50={pct(d['vis_total_lat_ms'],.5):.0f} p95={pct(d['vis_total_lat_ms'],.95):.0f}")

    auto = d["tur_auto"]
    pe = d["tur_pos_err"]; pw = d["tur_power"]; vel = d["tur_vel_filt"]
    idx = [i for i in range(n) if auto[i] == 1]
    print(f"\n== TURRET (auto-aim loops: {len(idx)}) ==")
    if idx:
        a = [abs(pe[i]) for i in idx]
        print(f"   |pos err| ticks p50={pct(a,.5):.2f} p90={pct(a,.9):.2f} max={max(a):.1f}")
        stuck = [i for i in idx if abs(pe[i]) > 2 and abs(pw[i]) < 0.12 and abs(vel[i]) < 10]
        print(f"   STALL-ish (err>2 ticks, power<0.12, ~no motion): {len(stuck)} loops ({100*len(stuck)/len(idx):.1f}%)  -> raise STICTION_POWER/KI if high")
        sat = sum(d["tur_saturated"][i] for i in idx)
        print(f"   power saturated: {100*sat/len(idx):.1f}% of auto loops")
        flips = sum(1 for a_, b_ in zip(idx, idx[1:]) if b_ == a_ + 1 and pe[a_]*pe[b_] < 0 and abs(pe[a_]) > 1.5)
        print(f"   large sign flips of err (overshoot/oscillation indicator): {flips}")
        lim = sum(1 for i in idx if abs(d['tur_ticks'][i]) >= 139)
        print(f"   loops at soft limit: {lim}")


    if "fus_resid_deg" in d:
        print("\n== ODOMETRY / FUSION (shadow unless fus_enabled=1) ==")
        if "odo_read_us" in d:
            rd = [x for x in d["odo_read_us"] if not math.isnan(x) and x > 0]
            print(f"   Pinpoint read: p50={pct(rd,.5)/1000:.2f} ms p95={pct(rd,.95)/1000:.2f} ms   history misses={d['odo_hist_miss'][-1]:.0f}")
        v = d["fus_valid"]; print(f"   fused target valid {100*sum(1 for x in v if x==1)/n:.0f}% of loops, reseeds={d['fus_reseeds'][-1]:.0f}")
        moving = lambda i: d["odo_speed"][i] > 5 or abs(d["odo_heading_rate"][i]) > 15
        for label, cond in (("robot STILL ", lambda i: not moving(i)), ("robot MOVING", moving)):
            rs = [abs(d["fus_resid_deg"][i]) for i in range(n) if not math.isnan(d["fus_resid_deg"][i]) and cond(i)]
            ri = [d["fus_resid_in"][i] for i in range(n) if not math.isnan(d["fus_resid_in"][i]) and cond(i)]
            if rs: print(f"   {label}: odometry-predicted bearing vs camera at capture time: |err| p50={pct(rs,.5):.2f} p90={pct(rs,.9):.2f} deg  |  field-target scatter p50={pct(ri,.5):.1f} p90={pct(ri,.9):.1f} in  (n={len(rs)})")
        print("   (small MOVING numbers = odometry is accurate enough to aim from; then set ODOMETRY_FUSION_ENABLED = true)")
        lead = [d["fus_minus_setpoint_ticks"][i] for i in range(n) if d["tur_auto"][i] == 1 and not math.isnan(d["fus_minus_setpoint_ticks"][i]) and moving(i)]
        if lead: print(f"   while moving in auto-aim: fused setpoint minus actual setpoint p50={pct([abs(x) for x in lead],.5):.1f} p90={pct([abs(x) for x in lead],.9):.1f} ticks (how much the fusion would change the aim)")
    print("\n== READY TIMING (from events) ==")
    ev = glob.glob(path.replace("_loop.csv", "_events.log"))
    if ev:
        for line in open(ev[0]):
            if any(k in line for k in ("AIM_READY","AIM_LOST","ROBOT_STOPPED","ROBOT_MOVING","AUTOAIM","TARGET_","SOFT_LIMIT","EXCEPTION","WARN")):
                print("  ", line.rstrip()[:200])

    if "odo_target_rate_dps" in d and "bearing_deg" in d:
        # Sign check needed before ODOMETRY_FEEDFORWARD_ENABLED: does odometry-predicted
        # bearing rate agree with the vision-measured bearing rate while moving?
        xs, ys = [], []
        for i in range(1, n):
            if d["vis_new_frame"][i] == 1 and d["odo_speed"][i] > 5 or abs(d["odo_heading_rate"][i]) > 10:
                db = d["bearing_deg"][i] - d["bearing_deg"][i-1]; dtt = (t[i]-t[i-1])/1000
                if dtt > 0 and not math.isnan(db) and abs(db/dtt) < 200:
                    xs.append(d["odo_target_rate_dps"][i]); ys.append(db/dtt)
        if len(xs) > 20:
            mx, my = st.mean(xs), st.mean(ys)
            num = sum((x-mx)*(y-my) for x, y in zip(xs, ys))
            den = math.sqrt(sum((x-mx)**2 for x in xs) * sum((y-my)**2 for y in ys)) or 1
            print(f"\n== ODOMETRY SIGN CHECK ==  corr(odo rate, vision bearing rate)={num/den:+.2f} over {len(xs)} samples "
                  "(>0.5 good, <-0.5 sign flipped, ~0 untrustworthy)")

if __name__ == "__main__":
    main()
