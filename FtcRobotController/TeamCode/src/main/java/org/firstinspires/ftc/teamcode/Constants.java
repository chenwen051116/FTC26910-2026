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

        // default turret angle = 0.05

        public static final double[] SHORT_RANGE_DISTANCE = {
                35.1465,
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
                2100,
                2125,
                2175,
                2275,
                2350,
                2500,
                2600,
                2675,
                2800,
                2900,
                3000,
                3050,
                3100,


        };

        public static final double[] SHORT_RANGE_HOOD_POSITION = {
                1,
                1,
                1,
                0.80,
                0.70,
                0.50,
                0.40,
                0.30,
                0.2,
                0,
                0,
                0,
                0

        };

        public static final double[] LONG_RANGE_DISTANCE = {
                130.1029,
                135.4787
        };

        public static final int LONG_RANGE_DISTANCE_INTERVAL = 5;

        public static final int[] LONG_RANGE_RPM = {
                3625,
                3700
        };

        public static final double[] LONG_RANGE_HOOD_POSITION = {
                0,
                0
        };

        public static final double LONGEST_SHORT_DISTANCE = 90;


    }
}
