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
        public static final double TURRET_OFFSET = 2.7138;

        public static final double[] SHORT_RANGE_DISTANCE = {
                50.7377,
                55.0823,
                60.2121,
                65.2985,
                70.5233,
                75.2409, // THIS POINT NEEDS TO BE DOUBLE-CHECKED
                80.1729,
                85.2679,
                90.2571,
                95.1201,
                100.2495,
                105.2593,
                110.0269
        };

        public static final int SHORT_RANGE_DISTANCE_INTERVAL = 5;

        public static final int[] SHORT_RANGE_RPM = {
                2500,
                2500,
                2500,
                2500,
                2550,
                2600,
                2700,
                2725,
                2750,
                2825,
                2950,
                3050,
                3175
        };

        public static final double[] SHORT_RANGE_HOOD_POSITION = {
                0.9,
                0.5,
                0.3,
                0.2,
                0.175,
                0.175,
                0.15,
                0.125,
                0.125,
                0.125,
                0.125,
                0.125,
                0.1125
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
