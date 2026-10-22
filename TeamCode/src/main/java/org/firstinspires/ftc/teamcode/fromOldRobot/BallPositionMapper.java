//package org.firstinspires.ftc.teamcode.fromOldRobot;
//
///**
// * Converts a Limelight tx/ty detection into a real-world (x, y) position,
// * relative to the robot chassis, using the known fixed height offset between
// * the camera and the ball plane (floor), the camera's fixed mounting tilt,
// * and the current pan angle of the Axon Max turret servo.
// */
//public class BallPositionMapper {
//
//    // ---------------- Physical constants -- MEASURE THESE ON YOUR ROBOT ----------------
//    // Vertical distance from the Limelight lens to the floor/ball plane, in inches (or cm).
//    static final double CAMERA_HEIGHT = 14.0; // EXAMPLE -- replace with your real measurement
//
//    // How many degrees the camera is tilted DOWN from horizontal at zero servo angle.
//    // If the Limelight is dead level, this is 0. If it's tilted down 20 degrees, use 20.
//    static final double CAMERA_MOUNT_ANGLE_DEG = 20.0; // EXAMPLE -- replace with your real value
//
//    /**
//     * Converts one detection's tx/ty into robot-relative (x, y) coordinates.
//     *
//     * @param tx             horizontal angle from Limelight, degrees (+ = right)
//     * @param ty             vertical angle from Limelight, degrees (per your setup, more
//     *                       negative = closer to bottom of frame / closer to robot)
//     * @param turretAngleDeg current Axon Max servo angle, degrees, relative to robot-forward
//     *                       (0 = camera pointed straight forward on the chassis)
//     * @return double[]{x, y} in robot-relative coordinates -- x = lateral (right +), y = forward
//     */
//    public static double[] toRobotRelative(double tx, double ty, double turretAngleDeg) {
//        // Total downward angle from horizontal, accounting for mount tilt + current ty reading.
//        // NOTE: verify this sign convention against real data -- place a ball at a known
//        // distance, read ty, and confirm the computed distance moves the right direction
//        // as you move the ball closer/farther. Flip the sign here if it doesn't match.
//        double totalVerticalAngleDeg = CAMERA_MOUNT_ANGLE_DEG - ty;
//        double totalVerticalAngleRad = Math.toRadians(totalVerticalAngleDeg);
//
//        // Forward distance from the camera to the ball, along the camera's current pointing direction.
//        double distanceForward = CAMERA_HEIGHT / Math.tan(totalVerticalAngleRad);
//
//        // Lateral distance relative to the camera's current pointing direction.
//        double distanceLateral = distanceForward * Math.tan(Math.toRadians(tx));
//
//        // Rotate that (forward, lateral) pair by the turret's current pan angle,
//        // to express it relative to the ROBOT CHASSIS rather than the camera's current heading.
//        double turretRad = Math.toRadians(turretAngleDeg);
//
//        double robotForward = distanceForward * Math.cos(turretRad) - distanceLateral * Math.sin(turretRad);
//        double robotLateral = distanceForward * Math.sin(turretRad) + distanceLateral * Math.cos(turretRad);
//
//        return new double[]{ robotLateral, robotForward }; // {x, y}
//    }
//}