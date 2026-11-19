package org.firstinspires.ftc.teamcode.tools;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.LinkedHashMap;
import java.util.Locale;
import java.util.Map;

/**
 * Saves/loads named robot poses in the robot controller's storage:
 *   /sdcard/FIRST/biobuzz_poses.csv   (name,x_in,y_in,heading_deg)
 *   /sdcard/FIRST/biobuzz_poses.java.txt (paste-ready Java)
 *
 * Pull to your computer with:  adb pull /sdcard/FIRST/biobuzz_poses.csv
 */
public class PoseStore {
    public static final File CSV = new File(AppUtil.FIRST_FOLDER, "biobuzz_poses.csv");
    public static final File JAVA = new File(AppUtil.FIRST_FOLDER, "biobuzz_poses.java.txt");
    /** Append-only history: stamp,name,x,y,h -- one row per recorded pose, never overwritten. */
    public static final File LOG = new File(AppUtil.FIRST_FOLDER, "biobuzz_pose_log.csv");

    /** Timestamp shared by every save in one OpMode run; set by beginSession(). */
    private static String session = null;

    private static String stamp() {
        return session != null ? session
                : new SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(new Date());
    }

    /** Call in init(): groups this run's appends under one timestamp. */
    public static void beginSession() {
        session = new SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(new Date());
    }

    /** Append one recorded pose to the history log. @return null on success. */
    public static String appendOne(String name, P p) {
        try {
            AppUtil.FIRST_FOLDER.mkdirs();
            boolean fresh = !LOG.exists();
            try (FileWriter w = new FileWriter(LOG, true)) {
                if (fresh) w.write("stamp,name,x,y,h\n");
                w.write(String.format(Locale.US, "%s,%s,%.2f,%.2f,%.1f\n",
                        stamp(), name, p.x, p.y, p.headingDeg));
            }
            return null;
        } catch (IOException ex) {
            return ex.getMessage();
        }
    }

    public static class P {
        public final double x, y, headingDeg;
        public P(double x, double y, double headingDeg) {
            this.x = x; this.y = y; this.headingDeg = headingDeg;
        }
    }

    /** Insertion-ordered map of name -> pose. */
    public static Map<String, P> load() {
        Map<String, P> m = new LinkedHashMap<>();
        if (!CSV.exists()) return m;
        try (BufferedReader r = new BufferedReader(new FileReader(CSV))) {
            String line;
            while ((line = r.readLine()) != null) {
                String[] t = line.split(",");
                if (t.length < 4) continue;
                try {
                    m.put(t[0].trim(), new P(Double.parseDouble(t[1]),
                            Double.parseDouble(t[2]), Double.parseDouble(t[3])));
                } catch (NumberFormatException ignored) { /* header/bad line */ }
            }
        } catch (IOException ignored) { }
        return m;
    }

    /** @return null on success, otherwise an error message. */
    public static String save(Map<String, P> poses) {
        try {
            AppUtil.FIRST_FOLDER.mkdirs();
            try (FileWriter w = new FileWriter(CSV)) {
                for (Map.Entry<String, P> e : poses.entrySet()) {
                    P p = e.getValue();
                    w.write(String.format(java.util.Locale.US, "%s,%.2f,%.2f,%.1f\n",
                            e.getKey(), p.x, p.y, p.headingDeg));
                }
            }
            try (FileWriter w = new FileWriter(JAVA)) {
                for (Map.Entry<String, P> e : poses.entrySet()) {
                    P p = e.getValue();
                    w.write(String.format(java.util.Locale.US,
                            "private final Pose %s = poseFactory.of(%.1f, %.1f, %.0f);\n",
                            e.getKey(), p.x, p.y, p.headingDeg));
                }
            }
            return null;
        } catch (IOException ex) {
            return ex.getMessage();
        }
    }
}
