#!/usr/bin/env python3 -I
"""
Turn ODO CALIBRATION marks + the TRUE physical values you measured into exact
OdometryConfig corrections.

  analyze_odometry_log.py <run_dir|*_events.log> [--truth NAME=value ...]

Truth values (what you physically measured):
  STRAIGHT_FWD=96       inches pushed straight
  STRAIGHT_STRAFE=96    inches pushed sideways
  SPIN_CCW=3600         total degrees turned (10 turns CCW = 3600)
  SPIN_CW=3600          total degrees turned (magnitude; sign handled)
  LOOP_RETURN=0         (no truth needed: closure error is reported)
  HOLD_STILL=0          (no truth needed: drift is reported)
"""
import glob, math, os, re, sys

IN_PER_MM = 1 / 25.4

def parse(path):
    if os.path.isdir(path):
        files = sorted(glob.glob(os.path.join(path, "*odocal_events.log")))
        if not files: sys.exit("no *odocal_events.log in " + path)
        path = files[-1]
    cfg = {}; marks = []
    for line in open(path):
        m = re.search(r"ODO_CONFIGURED .*offsets=\(([-\d.]+),([-\d.]+)\)in ticksPerMm=([-\d.]+) yawScalar=([-\d.]+)", line)
        if m: cfg = dict(xoff=float(m[1]), yoff=float(m[2]), tpm=float(m[3]), yaw=float(m[4]))
        m = re.search(r"(MARK_START|MARK_END) (.*)", line)
        if m:
            kv = dict(re.findall(r"(\w+)=([^\s]+)", m[2])); kv["kind"] = m[1]; marks.append(kv)
        if "ODO_WARNING" in line or "REZERO" in line or "EXCEPTION" in line: print("NOTE:", line.strip()[:160])
        if "AUTOCAL_" in line or "AUTOSPIN_" in line or "HUB_IMU" in line: print("AUTO:", line.strip()[:220])
    return path, cfg, marks

def auto_analyze(csv_path):
    """No A/B marks needed: pod health + per-turn spin table straight from the loop CSV."""
    import csv
    rows = [r for r in csv.DictReader(open(csv_path)) if None not in r.values()]
    n = len(rows); f = lambda i, k: float(rows[i][k]); T = [(f(i, "t_ms") - f(0, "t_ms")) / 1000 for i in range(n)]
    names = ["NOT_READY", "READY", "CALIBRATING", "FAULT_X_POD_NOT_DETECTED", "FAULT_Y_POD_NOT_DETECTED",
             "FAULT_NO_PODS_DETECTED", "FAULT_IMU_RUNAWAY", "FAULT_BAD_READ"]
    print("\n== POD / BUS HEALTH ==  (%d loops, %.0f s)" % (n, T[-1]))
    cnt = {}; trans = {}; prev = None
    for i in range(n):
        s_ = int(f(i, "status")); cnt[s_] = cnt.get(s_, 0) + 1
        if prev is not None and s_ != prev: trans[s_] = trans.get(s_, 0) + 1
        prev = s_
    for k in sorted(cnt): print("   %-26s %6d loops (%4.1f%%)  entered %d times" % (names[k] if k < 8 else k, cnt[k], 100 * cnt[k] / n, trans.get(k, 0)))
    jumps = []
    for i in range(1, n):
        dt = T[i] - T[i - 1]
        j = math.hypot(f(i, "x_in") - f(i - 1, "x_in"), f(i, "y_in") - f(i - 1, "y_in"))
        if dt < 0.2 and j > 2.0 + 80 * dt: jumps.append((T[i], j))
    print("   impossible pose jumps: %d  (max %.1f in)" % (len(jumps), max([j for _, j in jumps], default=0)))
    rd = sorted(f(i, "read_us") for i in range(n)); print("   Pinpoint read time: p50 %.1f ms  p95 %.1f ms" % (rd[n // 2] / 1000, rd[int(n * .95)] / 1000))
    if cnt.get(3, 0) or cnt.get(4, 0) or cnt.get(5, 0):
        print("   >>> A POD IS DROPPING OUT. Reseat/inspect that pod's cable at BOTH ends before calibrating anything. <<<")

    # stationary drift
    best = None
    for i in range(n):
        if abs(f(i, "vel_h_dps")) < 0.5 and math.hypot(f(i, "vel_x"), f(i, "vel_y")) < 0.5:
            if best is None: best = i
        else:
            if best is not None and T[i - 1] - T[best] > 20:
                print("   still for %.0f s at t=%.0f: heading drift %.3f deg, position drift %.3f in" % (T[i-1]-T[best], T[best], f(i-1,"heading_unwrapped_deg")-f(best,"heading_unwrapped_deg"), math.hypot(f(i-1,"x_in")-f(best,"x_in"), f(i-1,"y_in")-f(best,"y_in"))))
            best = None
    if best is not None and T[-1] - T[best] > 20:
        print("   still for %.0f s at t=%.0f: heading drift %.3f deg, position drift %.3f in" % (T[-1]-T[best], T[best], f(n-1,"heading_unwrapped_deg")-f(best,"heading_unwrapped_deg"), math.hypot(f(n-1,"x_in")-f(best,"x_in"), f(n-1,"y_in")-f(best,"y_in"))))

    # per-turn spin table (rate > 60 deg/s, status READY only)
    tpm = 19.89436789
    print("\n== SPINS: pod ticks per full turn (implied pod offset = distance / angle) ==")
    print("   X offset should match Pedro xPodOffset, Y offset Pedro yPodOffset. dEncX of exactly 0 for whole turns = X pod not reading.")
    segs = []; cur = []
    for i in range(n):
        if abs(f(i, "vel_h_dps")) > 60 and int(f(i, "status")) == 1: cur.append(i)
        else:
            if len(cur) > 200: segs.append(cur)
            cur = []
    if len(cur) > 200: segs.append(cur)
    all_y = []; all_x = []
    for seg in segs:
        prev = seg[0]; k = 0; ys = []; xs = []
        for i in seg:
            if abs(f(i, "heading_unwrapped_deg") - f(prev, "heading_unwrapped_deg")) >= 360:
                dh = math.radians(f(i, "heading_unwrapped_deg") - f(prev, "heading_unwrapped_deg"))
                ex = (f(i, "enc_x") - f(prev, "enc_x")) / tpm; ey = (f(i, "enc_y") - f(prev, "enc_y")) / tpm
                xs.append(-ex / 25.4 / dh); ys.append(-ey / 25.4 / dh); k += 1; prev = i   # Y pod is REVERSED in Pedro: logged ticks are raw, so post-direction = -raw
        if k:
            print("   spin t=%.0f-%.0fs  %s  %d turns: implied xOffset %.3f in (range %.3f..%.3f), yOffset %.3f in (range %.3f..%.3f)" % (T[seg[0]], T[seg[-1]], "CCW" if f(seg[-1], "heading_unwrapped_deg") > f(seg[0], "heading_unwrapped_deg") else "CW ", k, sum(xs)/k, min(xs), max(xs), sum(ys)/k, min(ys), max(ys)))
            all_x += xs; all_y += ys
    if all_y:
        print("   OVERALL implied: xOffset %.3f in, yOffset %.3f in  (Y sign assumes yPodDirection REVERSED; compare with Pedro config)" % (sum(all_x)/len(all_x), sum(all_y)/len(all_y)))
        spread = (max(all_y) - min(all_y)) / max(abs(sum(all_y)/len(all_y)), 1e-6)
        if spread > 0.2: print("   NOTE: Y offset varies %.0f%% turn-to-turn: pod wheel contact / spin wobble; get X pod healthy and repeat before trusting offsets." % (spread * 100))

def fit_offsets(rows, cfg):
    """
    Offline replica of the Pinpoint dead reckoning (verified to 0.00 in RMS against logged poses):
        forward = dX_mm + xOff*dTheta ;  left = dY_post_mm - yOff*dTheta   (Y pod REVERSED => post = -raw)
    An offset error shows up as a circular wobble of the pose during a spin, so we search for the
    offsets that minimise the wobble around a smooth drift curve, block by block. Agreement between
    blocks (small sd) means the answer is trustworthy.
    """
    n = len(rows); f = lambda i, k: float(rows[i][k]); T = [(f(i, "t_ms") - f(0, "t_ms")) / 1000 for i in range(n)]
    ex = [f(i, "enc_x") for i in range(n)]; ey = [f(i, "enc_y") for i in range(n)]
    hd = [math.radians(f(i, "heading_unwrapped_deg")) for i in range(n)]
    tpm = cfg["tpm"]
    def integ(a, b, xo, yo, step):
        x = y = 0.0; out = []
        for i in range(a + 1, b + 1):
            if abs(ex[i] - ex[i-1]) > 5000 or abs(ey[i] - ey[i-1]) > 5000: continue
            dth = hd[i] - hd[i-1]
            fwd = (ex[i] - ex[i-1]) / tpm + xo * 25.4 * dth
            left = -(ey[i] - ey[i-1]) / tpm - yo * 25.4 * dth
            h = (hd[i] + hd[i-1]) / 2
            x += (fwd * math.cos(h) - left * math.sin(h)) / 25.4; y += (fwd * math.sin(h) + left * math.cos(h)) / 25.4
            if (i - a) % step == 0: out.append((T[i], x, y))
        return out
    def polyres(ts, v, deg=3):
        m = len(ts); t0 = ts[0]; tn = [(t - t0) / (ts[-1] - t0 + 1e-9) for t in ts]
        A = [[t ** k for k in range(deg + 1)] for t in tn]
        N = [[sum(A[i][p] * A[i][q] for i in range(m)) for q in range(deg + 1)] for p in range(deg + 1)]
        b = [sum(A[i][p] * v[i] for i in range(m)) for p in range(deg + 1)]
        for p in range(deg + 1):
            for q in range(p + 1, deg + 1):
                fc = N[q][p] / N[p][p]
                for c in range(deg + 1): N[q][c] -= fc * N[p][c]
                b[q] -= fc * b[p]
        co = [0] * (deg + 1)
        for p in range(deg, -1, -1): co[p] = (b[p] - sum(N[p][c] * co[c] for c in range(p + 1, deg + 1))) / N[p][p]
        return sum((v[i] - sum(co[k] * A[i][k] for k in range(deg + 1))) ** 2 for i in range(m))
    def wob(a, b, xo, yo):
        o = integ(a, b, xo, yo, 4)
        if len(o) < 20: return 1e9
        ts = [p[0] for p in o]
        return math.sqrt((polyres(ts, [p[1] for p in o]) + polyres(ts, [p[2] for p in o])) / len(o))
    # spin blocks of ~4 turns
    blocks = []; cur = []
    for i in range(n):
        if abs(f(i, "vel_h_dps")) > 60 and int(f(i, "status")) == 1: cur.append(i)
        else:
            if len(cur) > 200: blocks.append(cur)
            cur = []
    if len(cur) > 200: blocks.append(cur)
    jobs = []
    for seg in blocks:
        a = seg[0]
        for i in seg:
            if abs(hd[i] - hd[a]) >= math.radians(1440): jobs.append((a, i)); a = i
        if abs(hd[seg[-1]] - hd[a]) >= math.radians(720): jobs.append((a, seg[-1]))
    if not jobs: print("\n== OFFSET FIT == no spin of >=2 turns found"); return
    print("\n== OFFSET FIT (offline replica, %d spin blocks) ==" % len(jobs))
    res = []
    for a, b in jobs[:6]:
        best = None
        for xo in [i * 0.1 for i in range(-20, 21)]:
            for yo in [i * 0.1 for i in range(-30, 21)]:
                w = wob(a, b, xo, yo)
                if best is None or w < best[0]: best = (w, xo, yo)
        w0 = wob(a, b, cfg["xoff"], cfg["yoff"])
        for xo in [best[1] + i * 0.02 for i in range(-5, 6)]:
            for yo in [best[2] + i * 0.02 for i in range(-5, 6)]:
                w = wob(a, b, xo, yo)
                if w < best[0]: best = (w, xo, yo)
        print("   t=%5.1f-%5.1fs  wobble with config %.2f in -> best xOff %+.2f yOff %+.2f (%.2f in)" % (T[a], T[b], w0, best[1], best[2], best[0]))
        res.append(best)
    mx = sum(r[1] for r in res) / len(res); my = sum(r[2] for r in res) / len(res)
    sx = (sum((r[1] - mx) ** 2 for r in res) / len(res)) ** .5; sy = (sum((r[2] - my) ** 2 for r in res) / len(res)) ** .5
    print("   FIT: xPodOffset = %+.2f (sd %.2f)   yPodOffset = %+.2f (sd %.2f)   [config now %+.3f / %+.3f]" % (mx, sx, my, sy, cfg["xoff"], cfg["yoff"]))
    print("   trust it if sd < ~0.2 and the wobble with the fitted offsets is well below the config wobble.")

def auto_legs(rows, truth_in, cfg):
    """Detect straight forward/back drives (stick held, coast to a stop) and compare to a known leg length."""
    import statistics as st
    n = len(rows); f = lambda i, k: float(rows[i][k]); T = [(f(i, "t_ms") - f(0, "t_ms")) / 1000 for i in range(n)]
    legs = []; i = 0
    mv = lambda i: abs(f(i, "in_drive")) > 0.1 and abs(f(i, "in_strafe")) < 0.15 and abs(f(i, "in_turn")) < 0.15
    while i < n:
        if mv(i):
            a = i; j = i
            while j < n and abs(f(j, "in_drive")) > 0.02: j += 1
            k = j
            while k < n - 1 and math.hypot(f(k, "vel_x"), f(k, "vel_y")) > 1.5: k += 1
            d = math.hypot(f(k, "x_in") - f(a, "x_in"), f(k, "y_in") - f(a, "y_in"))
            if d > 0.5 * truth_in: legs.append(d)
            i = k + 1
        else: i += 1
    print("\n== STRAIGHT LEGS (assumed true length %.1f in each) ==" % truth_in)
    if len(legs) < 2: print("   fewer than 2 long straight legs found"); return
    m = st.mean(legs); sd = st.pstdev(legs); ratio = m / truth_in
    print("   %d legs: %s" % (len(legs), " ".join("%.1f" % d for d in legs)))
    print("   mean %.2f in (sd %.2f) -> reported/true = %.4f  (%s by %.1f%%)" % (m, sd, ratio, "SHORT" if ratio < 1 else "LONG", abs(1 - ratio) * 100))
    print("   consistent legs (sd %.1f%%) only prove repeatability; they match the truth only if each leg really was %.0f in." % (100 * sd / m, truth_in))
    print("   -> c.ticksPerUnit.set(OptionalDouble.of(%.3f)); c.encoderResolutionUnit.set(DistanceUnit.MM);   (ticks/mm in force: %.4f)" % (cfg["tpm"] * ratio, cfg["tpm"]))

def main():
    tpm_override = None
    if "--tpm" in sys.argv:
        tpm_override = float(sys.argv[sys.argv.index("--tpm") + 1])
    spinres = {}
    if "--spinres" in sys.argv:
        i = sys.argv.index("--spinres")
        for a in sys.argv[i+1:]:
            if a.startswith("--"): break
            k, v = a.split("="); spinres[k.upper()] = float(v)
    legs_truth = None
    if "--legs" in sys.argv:
        legs_truth = float(sys.argv[sys.argv.index("--legs") + 1])
    args = [a for a in sys.argv[1:] if a != "--auto"]
    if "--tpm" in args:
        i = args.index("--tpm"); args = args[:i] + args[i+2:]
    if "--spinres" in args:
        i = args.index("--spinres"); j = i + 1
        while j < len(args) and not args[j].startswith("--"): j += 1
        args = args[:i] + args[j:]
    if legs_truth is not None:
        args = [a for a in args if a != "--legs" and a != str(sys.argv[sys.argv.index("--legs") + 1])]
    truth = {}
    if "--truth" in args:
        i = args.index("--truth")
        for a in args[i+1:]:
            if a.startswith("--"): break
            k, v = a.split("="); truth[k.upper()] = float(v)
        args = args[:i]
    path, cfg, marks = parse(args[0] if args else ".")
    print("FILE", path)
    if not cfg: print("WARNING: no ODO_CONFIGURED line; assuming factory 19.8944 ticks/mm, yaw 1.0, offsets from config"); cfg = dict(xoff=float('nan'), yoff=float('nan'), tpm=19.89436789, yaw=1.0)
    print("config in force: xOff=%.4f yOff=%.4f in  ticks/mm=%.5f  yawScalar=%.5f" % (cfg["xoff"], cfg["yoff"], cfg["tpm"], cfg["yaw"]))
    if tpm_override:
        print(">>> --tpm %.5f: pod offsets below are re-fitted AS IF that resolution were in force (distances from marks are rescaled too)" % tpm_override)
        cfg_in_force_tpm = cfg["tpm"]; cfg["tpm"] = tpm_override

    loopcsv = path.replace("_events.log", "_loop.csv")
    if "--auto" in sys.argv or not any(m["kind"] == "MARK_END" for m in marks):
        if os.path.exists(loopcsv):
            auto_analyze(loopcsv)
            import csv as _csv
            if legs_truth is not None:
                auto_legs([r for r in _csv.DictReader(open(loopcsv)) if None not in r.values()], legs_truth, cfg if cfg else dict(xoff=0.644, yoff=-1.571, tpm=19.89436789))
            fit_offsets([r for r in _csv.DictReader(open(loopcsv)) if None not in r.values()], cfg if cfg else dict(xoff=0.644, yoff=-1.571, tpm=19.89436789))
    ends = [m for m in marks if m["kind"] == "MARK_END"]
    lin, yaw, xoffs, yoffs, strafe = [], [], [], [], []
    for m in ends:
        t = m["test"]; f = lambda k: float(m[k])
        d = math.hypot(f("dx"), f("dy")); dh = f("dHdgDeg")
        print("\n[%s #%s] dur %.1fs  reported: dx=%.3f dy=%.3f dist=%.3f in  dHdg=%.3f deg  dEnc=(%d,%d)" % (t, m["idx"], f("durS"), f("dx"), f("dy"), d, dh, int(m["dEncX"]), int(m["dEncY"])))
        tv = truth.get(t)
        key = "CCW" if t == "SPIN_CCW" else "CW" if t == "SPIN_CW" else None
        if key and key in spinres:
            r_ = spinres[key]                       # degrees PAST(+)/SHORT(-) of the start line, in the spin direction
            n_ = round((abs(dh) - r_) / 360.0)
            tv = 360.0 * n_ + r_
            print("   physical truth from your residual: %d turns %+.1f deg = %.1f deg" % (n_, r_, tv))
        if t in ("STRAIGHT_FWD", "STRAIGHT_STRAFE") and tv:
            ratio = d / tv
            print("   truth %.2f in -> reported/true = %.5f (%.2f%% %s)" % (tv, ratio, abs(ratio-1)*100, "long" if ratio > 1 else "short"))
            (lin if t == "STRAIGHT_FWD" else strafe).append(ratio)
        if t in ("SPIN_CCW", "SPIN_CW") and tv:
            signed = abs(tv) * (1 if t == "SPIN_CCW" else -1)
            ratio = dh / signed
            print("   truth %.1f deg -> reported/true = %.5f (%.3f%% %s)" % (signed, ratio, abs(ratio-1)*100, "over" if ratio > 1 else "under"))
            yaw.append(ratio)
            th = math.radians(signed)
            mm = lambda e: int(m[e]) / cfg["tpm"]
            xo = -mm("dEncX") * IN_PER_MM / th; yo = mm("dEncY") * IN_PER_MM / th
            print("   implied pod offsets from ticks: xOffset=%.3f in  yOffset=%.3f in   (position left behind after spin: dx=%.2f dy=%.2f in)" % (xo, yo, f("dx"), f("dy")))
            xoffs.append(xo); yoffs.append(yo)
        if t == "LOOP_RETURN":
            print("   closure error: %.2f in position, %.2f deg heading  -> %s" % (d, ((dh + 180) % 360) - 180, "GOOD (<1 in)" if d < 1 else "needs calibration"))
        if t == "HOLD_STILL":
            mins = f("durS") / 60
            print("   drift per minute: heading %.3f deg/min, position %.3f in/min" % (dh / mins, d / mins))

    print("\n================ RECOMMENDED CHANGES (offsets + ticks go in PedroPathing/Constants.java, YAW_SCALAR in OdometryConfig.java) ================")
    if lin:
        r = sum(lin) / len(lin); new = cfg["tpm"] * r
        print("Constants.localizerConfig: c.ticksPerUnit.set(OptionalDouble.of(%.5f)); c.encoderResolutionUnit.set(DistanceUnit.MM);   // forward scale error was %+.2f%%" % (new, (r-1)*100))
        if strafe:
            rs = sum(strafe) / len(strafe)
            print("   strafe scale error is %+.2f%% (forward %+.2f%%): " % ((rs-1)*100, (r-1)*100) +
                  ("one scalar fits both." if abs(rs-r) < 0.015 else "they differ >1.5%: a single scalar cannot fix both; check pod wheel contact/mounting."))
    else: print("(no STRAIGHT_FWD truth given: ticks/mm unchanged)")
    if yaw:
        r = sum(yaw) / len(yaw); new = cfg["yaw"] / r
        print("YAW_SCALAR = %.5f;   // IMU %s by %.3f%%" % (new, "over-reported" if r > 1 else "under-reported", abs(r-1)*100))
    else: print("(no SPIN truth given: yaw scalar unchanged)")
    if xoffs:
        xo, yo = sum(xoffs)/len(xoffs), sum(yoffs)/len(yoffs)
        print("Pedro Constants xPodOffset ~ %.3f (now %.3f)   yPodOffset ~ %.3f (now %.3f)" % (xo, cfg["xoff"], yo, cfg["yoff"]))
        print("   Use these only if they differ by >0.15 in; confirm by re-running a spin test: the pose should return to where it started.")
        if abs(xo + cfg["xoff"]) < 0.2 and abs(xo - cfg["xoff"]) > 0.4: print("   NOTE: implied X offset looks sign-flipped relative to config.")
        if abs(yo + cfg["yoff"]) < 0.2 and abs(yo - cfg["yoff"]) > 0.4: print("   NOTE: implied Y offset looks sign-flipped relative to config.")

if __name__ == "__main__":
    main()
