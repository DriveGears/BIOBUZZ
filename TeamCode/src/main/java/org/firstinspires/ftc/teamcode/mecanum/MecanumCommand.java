package org.firstinspires.ftc.teamcode.mecanum;


import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.PinPointOdometrySubsystem;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Command wrapper for controlling a Mecanum drive system using
 * positional PID and field-oriented driving.
 */
public class MecanumCommand {
    public boolean isHeadingStale() { return pinPointOdoSubsystem.isStale(); }
    public void    stopOdometry()   { pinPointOdoSubsystem.stopThread(); }
    private MecanumSubsystem mecanumSubsystem;
    private PinPointOdometrySubsystem pinPointOdoSubsystem;
    private Hardware hw;
    private ElapsedTime elapsedTime;

    public double xFinal, yFinal, thetaFinal;
    public double velocity;

    private double ex = 0;
    private double ey = 0;
    private double etheta = 0;

    public MecanumCommand(Hardware hw) {
        this.hw = hw;
        this.mecanumSubsystem = new MecanumSubsystem(hw);
        this.pinPointOdoSubsystem = new PinPointOdometrySubsystem(hw);
        elapsedTime = new ElapsedTime();
        xFinal     = pinPointOdoSubsystem.getX();
        yFinal     = pinPointOdoSubsystem.getY();
        thetaFinal = pinPointOdoSubsystem.getHeading();
        velocity   = 0;
        turnOffInternalPID();
    }

    public void disableOdoLogging() {
        pinPointOdoSubsystem.disableLogging();
    }

    /** Flush and close the odo_log CSV safely. Call after the OpMode loop exits. */
    public void closeOdoLog() {
        pinPointOdoSubsystem.closeLog();
    }

    public void setConstants(double kpx, double kdx, double kix,
                             double kpy, double kdy, double kiy,
                             double kptheta, double kdtheta, double kitheta) {
        MecanumConstants.kpx = kpx; MecanumConstants.kdx = kdx; MecanumConstants.kix = kix;
        MecanumConstants.kpy = kpy; MecanumConstants.kdy = kdy; MecanumConstants.kiy = kiy;
        MecanumConstants.kptheta = kptheta; MecanumConstants.kdtheta = kdtheta; MecanumConstants.kitheta = kitheta;
        mecanumSubsystem.updatePIDConstants();
    }

    public void turnOffInternalPID() {
        mecanumSubsystem.turnOffInternalPID();
    }

    public void processPIDUsingPinpoint() {
        ex     =  mecanumSubsystem.globalXControllerOutputPositional(xFinal, pinPointOdoSubsystem.getX());
        ey     = -mecanumSubsystem.globalYControllerOutputPositional(yFinal, pinPointOdoSubsystem.getY());
        etheta =  mecanumSubsystem.globalThetaControllerOutputPositional(thetaFinal, pinPointOdoSubsystem.getHeading());

        double max = Math.max(Math.abs(ex), Math.abs(ey));
        if (max > velocity) {
            double scalar = velocity / max;
            ex *= scalar; ey *= scalar; etheta *= scalar;
        }
        moveGlobalPartialPinPoint(ex, ey, etheta);
    }

    public void moveGlobalPartialPinPoint(double vertical, double horizontal, double rotational) {
        double angle          = Math.PI / 2 - pinPointOdoSubsystem.getHeading();
        double localVertical  = vertical   * Math.cos(pinPointOdoSubsystem.getHeading()) - horizontal * Math.cos(angle);
        double localHorizontal = vertical  * Math.sin(pinPointOdoSubsystem.getHeading()) + horizontal * Math.sin(angle);
        mecanumSubsystem.partialMove(localVertical, localHorizontal, rotational);
    }

    /** Full reset — zeros XY position AND resets IMU heading to 0. */
    public void resetPinPointOdometry() {
        pinPointOdoSubsystem.reset();
    }

    /**
     * Position-only reset — zeros XY while preserving the current heading.
     * Use this for mid-match re-zeroing so the turret's world-angle tracking
     * is not corrupted by a sudden heading jump.
     */
    public void resetPositionOnly() {
        pinPointOdoSubsystem.resetPositionOnly();
    }

    public boolean moveToPos(double x, double y, double theta) {
        elapsedTime.reset();
        setFinalPosition(30, x, y, theta);
        return isPositionReached();
    }

    public void setFinalPosition(double velocity, double x, double y, double theta) {
        this.xFinal     = x;
        this.yFinal     = y;
        this.thetaFinal = theta;
        this.velocity   = velocity;
    }

    public boolean isPositionReached()    { return isXReached() && isYReached() && isThetaReached(); }
    public double getXDifferencePinPoint(){ return Math.abs(xFinal - pinPointOdoSubsystem.getX()); }
    public double getYDifferencePinPoint(){ return Math.abs(yFinal - pinPointOdoSubsystem.getY()); }
    public double getThetaDifferencePinPoint() { return Math.abs(thetaFinal - pinPointOdoSubsystem.getHeading()); }
    public boolean isYReached()    { return getYDifferencePinPoint()     < 2.5; }
    public boolean isXReached()    { return getXDifferencePinPoint()     < 2.5; }
    public boolean isThetaReached(){ return getThetaDifferencePinPoint() < 0.1; }
    public boolean isThetaPassed() { return getThetaDifferencePinPoint() < 0.22; }
    public boolean isXPassed()     { return getXDifferencePinPoint()     < 10; }
    public boolean isYPassed()     { return getYDifferencePinPoint()     < 10; }

    public double getOdoX()       { return pinPointOdoSubsystem.getX(); }
    public double getOdoY()       { return pinPointOdoSubsystem.getY(); }
    public double getOdoHeading() { return pinPointOdoSubsystem.getHeading(); }

    public double fieldOrientedMove(double vertical, double horizontal, double rotational) {
        mecanumSubsystem.fieldOrientedMove(-vertical, horizontal, rotational, pinPointOdoSubsystem.getHeading());
        return pinPointOdoSubsystem.getHeading();
    }

    public void motorProcess() {
        processPIDUsingPinpoint();
        mecanumSubsystem.motorProcessNoEncoder();
    }

    public double normalMove(double vertical, double horizontal, double rotational) {
        mecanumSubsystem.normalMove(vertical, horizontal, rotational, pinPointOdoSubsystem.getHeading());
        return pinPointOdoSubsystem.getHeading();
    }

    public int getRawX() {
        if (this.pinPointOdoSubsystem != null) {
            return (int) this.pinPointOdoSubsystem.getRawX();
        }
        return 0;
    }

    public int getRawY() {
        if (this.pinPointOdoSubsystem != null) {
            return (int) this.pinPointOdoSubsystem.getRawY();
        }
        return 0;
    }

    public void stop() { mecanumSubsystem.stop(true); }

    public double getHeadingVelocity() { return pinPointOdoSubsystem.getHeadingVelocity(); }

    public void processOdometry() { pinPointOdoSubsystem.processOdometry(); }

    public double getX() { return pinPointOdoSubsystem.getX(); }
    public double getY() { return pinPointOdoSubsystem.getY(); }
}
