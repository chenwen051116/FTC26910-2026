package org.firstinspires.ftc.teamcode;


public class Constants {
    public static final double TICKS_PER_REVOLUTION = 28; // ticks per revolution = TICKS_PER_REVOLUTION * MOTOR_GEAR_RATIO
    public static final double ENCODER_TICKS_PER_REVOLUTION = -8192;

    public static class Shooter {
        public static final double TURRET_GEAR_RATIO = 120.0 / 20.0;
        public static final double FLYWHEEL_GEAR_RATIO = 1.0;

        // TODO: find these constants experimentally
        public static final double C_VX = 0;
        public static final double C_VY = 0;
        public static final double C_AX = 0;
        public static final double C_AY = 0;
        public static final double TURRET_OFFSET = 2.7138;

        public static final double[] SHORT_RANGE_DISTANCE = {
                55.1936,
                60.2264,
                65.4608,
                70.2686,
                75.2001,
                80.3232,
                85.2354,
                90.3840,
                95.2980,
                100.1824,
                105.2118,
                110.2972
        };

        public static final int SHORT_RANGE_DISTANCE_INTERVAL = 5;

        public static final int[] SHORT_RANGE_RPM = {
                2400,
                2415,
                2430,
                2475,
                2550,
                2650,
                2750,
                2850,
                3125,
                3200,
                3300,
                3350
        };

        public static final double[] SHORT_RANGE_HOOD_POSITION = {
                1,
                0.7,
                0.6,
                0.55,
                0.45,
                0.4,
                0.3,
                0.25,
                0.15,
                0.1,
                0,
                0

        };

        public static final double[] LONG_RANGE_DISTANCE = {
                138.0012,
                147.4787
        };

        public static final int LONG_RANGE_DISTANCE_INTERVAL = 5;

        public static final int[] LONG_RANGE_RPM = {
                3800,
                3900
        };

        public static final double[] LONG_RANGE_HOOD_POSITION = {
                0,
                0
        };

        public static final double LONGEST_SHORT_DISTANCE = 110;


    }
}
