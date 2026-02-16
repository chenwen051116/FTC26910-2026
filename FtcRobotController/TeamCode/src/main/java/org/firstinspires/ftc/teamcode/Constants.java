package org.firstinspires.ftc.teamcode;

public class Constants {
    public static final double MOTOR_TICKS_PER_MINUTE = 28 * 6000; // ticks per revolution = MOTOR_TICKS_PER_MINUTE / RPM
    public static final double SERVO_RANGE = Math.toRadians(270);

    public static class Shooter {
        public static final double HOOD_BASE_ANGLE = Math.toRadians(30);
        public static final double TURRET_GEAR_RATIO = 120.0 / 18.0;
        public static final double HOOD_GEAR_RATIO = 4.0;
        public static final double FLYWHEEL_GEAR_RATIO = 1.0;

        // TODO: find these constants experimentally
        public static final double T1 = 0; // the air time of the ball, which depends on the y-velocity
        public static final double X1 = 0; // the ratio of the x-velocity and the distance,
        public static final double Y1 = 0; // the y-velocity of the ball that corresponds to T1
    }
}
