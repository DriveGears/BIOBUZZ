package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.Date;
import java.util.LinkedHashMap;
import java.util.Locale;
import java.util.Map;

/**
 * On-robot flight recorder. Writes three files per run to the Control Hub at
 *   /sdcard/FIRST/biobuzz_logs/
 *     <stamp>_<name>_loop.csv     one row per OpMode loop (all numeric)
 *     <stamp>_<name>_events.log   timestamped human-readable events + exceptions
 *     <stamp>_<name>_config.txt   every tunable constant + run metadata
 *
 * Pull them with:  adb pull /sdcard/FIRST/biobuzz_logs ./logs
 *
 * Usage per loop:  rec.put("col", value); ...; rec.endRow();
 * The column set is fixed by the FIRST row; keys first seen later are dropped
 * (and reported once in the events log). Never throws: any I/O failure just
 * disables logging and is visible through {@link #getStatus()}.
 */
public class FlightRecorder {

    public static final String LOG_DIR_NAME = "biobuzz_logs";
    private static final int KEEP_SESSIONS = 40;
    private static final long FLUSH_INTERVAL_MS = 200;

    private final long startNanos = System.nanoTime();
    private final Map<String, Integer> columns = new LinkedHashMap<>();
    private double[] values = new double[256];
    private boolean headerWritten;
    private boolean ok;
    private String error = "";
    private long rows;
    private long lastFlushMs;
    private long droppedKeys;

    private BufferedWriter csv;
    private BufferedWriter events;
    private final String basePath;

    private FlightRecorder(String name) {
        String path = "(disabled)";
        try {
            File dir = new File(AppUtil.FIRST_FOLDER, LOG_DIR_NAME);
            //noinspection ResultOfMethodCallIgnored
            dir.mkdirs();
            pruneOldSessions(dir);
            String stamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
            String base = stamp + "_" + name.replaceAll("[^A-Za-z0-9_-]", "_");
            path = new File(dir, base).getAbsolutePath();
            csv = new BufferedWriter(new FileWriter(new File(dir, base + "_loop.csv")), 1 << 16);
            events = new BufferedWriter(new FileWriter(new File(dir, base + "_events.log")), 1 << 14);
            ok = true;
        } catch (Throwable t) {
            ok = false;
            error = String.valueOf(t);
        }
        basePath = path;
        Arrays.fill(values, Double.NaN);
        event("RECORDER_START ok=" + ok + (ok ? "" : " error=" + error));
    }

    /** Creates a recorder; never returns null and never throws. */
    public static FlightRecorder start(String name) {
        return new FlightRecorder(name);
    }

    /** Writes the config snapshot (tunables, build tag, anything useful for diagnosis). */
    public void writeConfig(Map<String, String> config) {
        if (!ok) return;
        try (BufferedWriter w = new BufferedWriter(new FileWriter(basePath + "_config.txt"))) {
            for (Map.Entry<String, String> e : config.entrySet()) {
                w.write(e.getKey() + " = " + e.getValue());
                w.newLine();
            }
        } catch (Throwable t) {
            event("CONFIG_WRITE_FAILED " + t);
        }
    }

    public double nowMs() {
        return (System.nanoTime() - startNanos) / 1e6;
    }

    public void put(String key, double value) {
        Integer idx = columns.get(key);
        if (idx == null) {
            if (headerWritten) {
                if (droppedKeys++ == 0) event("WARN column added after header, dropped: " + key);
                return;
            }
            idx = columns.size();
            if (idx >= values.length) {
                int old = values.length;
                values = Arrays.copyOf(values, old * 2);
                Arrays.fill(values, old, values.length, Double.NaN);
            }
            columns.put(key, idx);
        }
        values[idx] = value;
    }

    public void put(String key, boolean value) {
        put(key, value ? 1.0 : 0.0);
    }

    /** Commits the current row. Call once per OpMode loop, after every put(). */
    public void endRow() {
        if (!ok) return;
        try {
            if (!headerWritten) {
                StringBuilder h = new StringBuilder("t_ms");
                for (String k : columns.keySet()) h.append(',').append(k);
                csv.write(h.toString());
                csv.newLine();
                headerWritten = true;
            }
            StringBuilder sb = new StringBuilder(columns.size() * 8);
            double t = nowMs();
            sb.append(fmt(t));
            int n = columns.size();
            for (int i = 0; i < n; i++) {
                sb.append(',');
                double v = values[i];
                if (!Double.isNaN(v)) sb.append(fmt(v));
                values[i] = Double.NaN;
            }
            csv.write(sb.toString());
            csv.newLine();
            rows++;
            if (t - lastFlushMs >= FLUSH_INTERVAL_MS) {
                lastFlushMs = (long) t;
                csv.flush();
                events.flush();
            }
        } catch (Throwable th) {
            ok = false;
            error = String.valueOf(th);
        }
    }

    public void event(String message) {
        if (events == null) return;
        try {
            events.write(String.format(Locale.US, "%10.1f  %s", nowMs(), message));
            events.newLine();
        } catch (Throwable ignored) {
            // logging must never break the robot
        }
    }

    public void exception(String where, Throwable t) {
        StringWriter sw = new StringWriter();
        t.printStackTrace(new PrintWriter(sw));
        event("EXCEPTION in " + where + ": " + sw);
        flush();
    }

    public void flush() {
        try {
            if (csv != null) csv.flush();
            if (events != null) events.flush();
        } catch (Throwable ignored) {
        }
    }

    public void close() {
        event("RECORDER_CLOSE rows=" + rows);
        flush();
        try {
            if (csv != null) csv.close();
            if (events != null) events.close();
        } catch (IOException ignored) {
        }
        ok = false;
    }

    public String getStatus() {
        return ok ? "REC rows=" + rows + " " + basePath : "LOG OFF " + error;
    }

    public boolean isOk() {
        return ok;
    }

    private static String fmt(double v) {
        if (Double.isInfinite(v)) return v > 0 ? "1e9" : "-1e9";
        double r = Math.round(v * 1000.0) / 1000.0;
        if (r == Math.rint(r) && Math.abs(r) < 1e15) return Long.toString((long) r);
        return Double.toString(r);
    }

    private static void pruneOldSessions(File dir) {
        File[] files = dir.listFiles((d, n) -> n.endsWith("_loop.csv"));
        if (files == null || files.length < KEEP_SESSIONS) return;
        Arrays.sort(files);
        for (int i = 0; i < files.length - KEEP_SESSIONS + 1; i++) {
            String base = files[i].getName().replace("_loop.csv", "");
            //noinspection ResultOfMethodCallIgnored
            files[i].delete();
            //noinspection ResultOfMethodCallIgnored
            new File(dir, base + "_events.log").delete();
            //noinspection ResultOfMethodCallIgnored
            new File(dir, base + "_config.txt").delete();
        }
    }
}
