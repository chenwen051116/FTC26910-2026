package org.firstinspires.ftc.teamcode;


public class Constants {
    public static final double TICKS_PER_REVOLUTION = 28; // ticks per revolution = TICKS_PER_REVOLUTION * MOTOR_GEAR_RATIO

    public static class Shooter {
        // TODO: find these constants experimentally
        public static final double C_VX = 0;
        public static final double C_VY = 0;
        public static final double C_AX = 0;
        public static final double C_AY = 0;
        public static final double TURRET_OFFSET = 2.7138;

        public static final double[] SHORT_RANGE_DISTANCE = {
                40.1641,
                45.0128,
                50.2853,
                55.2221,
                60.0832,
                64.9781,
                70.0819,
                74.9951,
                80.0476,
                85.0591,
                90.0496,
                95.0801
        };

        public static final int SHORT_RANGE_DISTANCE_INTERVAL = 5;

        public static final int[] SHORT_RANGE_RPM = {
                2400,
                2480,
                2550,
                2620,
                2650,
                2680,
                2800,
                2980,
                3125,
                3200,
                3250,
                3300,
        };

        public static final double[] SHORT_RANGE_HOOD_POSITION = {
                1,
                1,
                0.85,
                0.65,
                0.55,
                0.45,
                0.3,
                0.2,
                0.1,
                0,
                0,
                0

        };

        public static final double[] LONG_RANGE_DISTANCE = {
                118.0012,
                127.4787
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

        public static final double LONGEST_SHORT_DISTANCE = 90;


    }
}
