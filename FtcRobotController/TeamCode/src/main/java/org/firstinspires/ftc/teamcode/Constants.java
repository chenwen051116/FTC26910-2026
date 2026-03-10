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
                2250,
                2275,
                2285,
                2300,
                2315,
                2335,
                2385,
                2420,
                2515,
                2685,
                2780,
                2985
        };

        public static final double[] SHORT_RANGE_HOOD_POSITION = {
                1,
                0.8,
                0.65,
                0.625,
                0.6,
                0.555,
                0.55,
                0.535,
                0.530,
                0.515,
                0.485,
                0.46
        };

        public static final double[] LONG_RANGE_DISTANCE = {
                140.0012,
                145.4787,
                150.6533,
                155.6174,
                160.1078
        };

        public static final int LONG_RANGE_DISTANCE_INTERVAL = 5;

        public static final int[] LONG_RANGE_RPM = {
                3465,
                3525,
                3575,
                3600,
                3650
        };

        public static final double[] LONG_RANGE_HOOD_POSITION = {
                0.29,
                0.25,
                0.24,
                0.23,
                0.215
        };

        public static final double LONGEST_SHORT_DISTANCE = 110;


    }
}
