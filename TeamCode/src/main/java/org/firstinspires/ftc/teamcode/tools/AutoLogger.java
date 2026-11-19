package org.firstinspires.ftc.teamcode.tools;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

/**
 * Writes a CSV log of an auto run to /sdcard/FIRST/redauto_log.csv
 * (overwritten each run). Pull with: adb pull /sdcard/FIRST/redauto_log.csv
 */
public class AutoLogger {
    public static final File FILE = new File(AppUtil.FIRST_FOLDER, "redauto_log.csv");
    private BufferedWriter w;
    private int n = 0;

    public void open() {
        try {
            AppUtil.FIRST_FOLDER.mkdirs();
            w = new BufferedWriter(new FileWriter(FILE, false));
            w.write("t_s,step,x,y,heading_deg,target_x,target_y,target_heading_deg,"
                    + "err_in,mode,shooter_target_rpm,shooter_rpm,aim_ready,intake,transfer,loop_ms\n");
        } catch (IOException e) {
            w = null;
        }
    }

    public void row(double t, String step, double x, double y, double hDeg,
                    double tx, double ty, double thDeg, String mode,
                    double shTarget, double shActual, boolean aim,
                    double intake, double transfer, double loopMs) {
        if (w == null) return;
        try {
            double err = Math.hypot(tx - x, ty - y);
            w.write(String.format(Locale.US,
                    "%.3f,%s,%.2f,%.2f,%.1f,%.2f,%.2f,%.1f,%.2f,%s,%.0f,%.0f,%d,%.2f,%.2f,%.1f\n",
                    t, step, x, y, hDeg, tx, ty, thDeg, err, mode,
                    shTarget, shActual, aim ? 1 : 0, intake, transfer, loopMs));
            if (++n % 5 == 0) w.flush();
        } catch (IOException ignored) { }
    }

    public void close() {
        if (w == null) return;
        try { w.flush(); w.close(); } catch (IOException ignored) { }
        w = null;
    }
}
