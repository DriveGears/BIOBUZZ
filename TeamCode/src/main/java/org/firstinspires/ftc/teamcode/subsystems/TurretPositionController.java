package org.firstinspires.ftc.teamcode.subsystems;

/**
 * Pure turret motor controller. Target selection, vision, odometry, limits,
 * and hardware writes deliberately live outside this class.
 */
final class TurretPositionController {

    static final class Output {
        final double proportional;
        final double damping;
        final double staticFriction;
        final double power;

        Output(double proportional, double damping, double staticFriction, double power) {
            this.proportional = proportional;
            this.damping = damping;
            this.staticFriction = staticFriction;
            this.power = power;
        }
    }

    Output calculate(
            double positionErrorTicks,
            double measuredVelocityTicksPerSecond,
            double positionKp,
            double velocityKd,
            double staticFrictionPower,
            double positionToleranceTicks) {
        if (Math.abs(positionErrorTicks) <= Math.max(0, positionToleranceTicks)) {
            return new Output(0, 0, 0, 0);
        }

        double proportional = positionKp * positionErrorTicks;
        double damping = -velocityKd * measuredVelocityTicksPerSecond;
        double staticFriction = staticFrictionPower == 0 ? 0
                : Math.copySign(Math.abs(staticFrictionPower), positionErrorTicks);
        return new Output(
                proportional,
                damping,
                staticFriction,
                proportional + damping + staticFriction);
    }
}
