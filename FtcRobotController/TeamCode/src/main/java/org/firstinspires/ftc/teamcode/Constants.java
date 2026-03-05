package org.firstinspires.ftc.teamcode;


public class Constants {
    public static final double TICKS_PER_REVOLUTION = 28; // ticks per revolution = TICKS_PER_REVOLUTION * MOTOR_GEAR_RATIO
    public static final double ENCODER_TICKS_PER_REVOLUTION = -8192;

    public static class Shooter {
        public static final double TURRET_GEAR_RATIO = 120.0 / 19.0;
        public static final double FLYWHEEL_GEAR_RATIO = 1.0;

        // TODO: find these constants experimentally
        public static final double C_VX = 0;
        public static final double C_VY = 0;
        public static final double C_AX = 0;
        public static final double C_AY = 0;

        public static final double[] SHORT_RANGE_DISTANCE = {
                50.6998,
                55.1927,
                60.0192,
                65.1723,
                70.0511,
                75.0961,
                80.6312,
                85.0506,
                90.1664,
                95.0526,
                100.7450,
                105.2476,
                110.2539
        };

        public static final int SHORT_RANGE_DISTANCE_INTERVAL = 5;

        public static final int[] SHORT_RANGE_RPM = {
                2500,
                2500,
                2500,
                2500,
                2500,
                2500,
                2600,
                2700,
                2750,
                2850,
                3100,
                3150,
                3200
        };

        public static final double[] SHORT_RANGE_HOOD_POSITION = {
                0.95,
                0.65,
                0.25,
                0.2,
                0.2,
                0.15,
                0.125,
                0.125,
                0.125,
                0.125,
                0.125,
                0.125,
                0.125
        };

        public static final double[] LONG_RANGE_DISTANCE = {

        };

        public static final int LONG_RANGE_DISTANCE_INTERVAL = 5;

        public static final int[] LONG_RANGE_RPM = {

        };

        public static final double[] LONG_RANGE_HOOD_POSITION = {

        };

        public static final double LONGEST_SHORT_DISTANCE = 110;
    }
}
