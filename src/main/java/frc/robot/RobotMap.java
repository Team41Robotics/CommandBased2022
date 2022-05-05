package frc.robot;

public final class RobotMap {
    public static final class Climber {

        public static double CLIMBING_DRIVE_MAX_SPEED = 0.4;
        public static double CLIMBING_DRIVING_SPEED_OFFSET = 4e-4;
        public static double CLIMBING_MAX_SPEED = 0.85;
        public static double CLIMBING_SPEED_SECOND_STAGE = 0.6;
        public static double CLIMBING_SLOW_SPEED = 0.4;
        public static double JOYSTICK_CURVE_POWER = 3; // x^3 function
        public static double JOYSTICK_DEADZONE = 0.1;
        public static double JOYSTICK_CLIMBING_MODE_DEADZONE = 0.001;
        public static int CLIMBING_PISTON_TIME_DELAY = 500; // ms
        public static int OUTER_GEAR_LOCK_ON = 4;
        public static int OUTER_GEAR_LOCK_OFF = 11;
        public static int INNER_GEAR_LOCK_ON = 10;
        public static int INNER_GEAR_LOCK_OFF = 5;
        public static int MIDDLE_ARM_RELEASE = 0;
        public static int MIDDLE_ARM_LOCK = 15;
        public static int MOVE_TO_INNER_ARM = 1;
        public static int MOVE_TO_OUTER_ARMS = 14;
        public static int LEFT_SOL_FWD = 3;
        public static int LEFT_SOL_RV = 12;
        public static int RIGHT_SOL_FWD = 2;
        public static int RIGHT_SOL_RV = 13;
        public static int PCM_PORT = 12;

        public static int CLIMBING_SPARK_F = 1;
        public static int CLIMBING_SPARK_B = 2;
        public static int FIRST_STAGE_LIMIT_SWITCH_R = 1;
        public static int FIRST_STAGE_LIMIT_SWITCH_L = 0;
        public static int FIRST_STAGE_LIMIT_SWTICH_M = 3;
        public static int SECOND_STAGE_LIMIT_SWITCH = 5;

    }
    public static final class driverStation{
            public static class SecondDriverStation {
        public static int INCREASE_HOOD_OFFSET = 11;
        public static int DECREASE_HOOD_OFFSET = 12;

        public static int MANUAL_SHOOTER_SPEED = 14;
        public static int SHOOTER_SPEED_SLIDER = 0;
        public static int LOW_GOAL_SETUP = 5;
        public static int AUTO_SHOOTING = 6;
        public static int SHOOTER_WARMUP = 15;

        public static int MANUAL_CLIMBING_TOGGLE = 13;
        public static int ENABLE_PISTON_BRAKE = 14;
        public static int CLIMBING_STATE_POV = 1;

        public static int HOOD_UP = 7;
        public static int HOOD_DOWN = 8;

        public static int SHOOTER_OFFSET_UP = 9;
        public static int SHOOTER_OFFSET_DOWN = 10;

        public static int FEED_BALL_TO_SHOOTER = 1;
    }

    public static class LeftJoy {
        public static int CLIMB_FWD = 3;
        public static int CLIMB_RV = 4;

        public static int INTAKE_PISTON_TOGGLE = 1;
    }

    public static class RightJoy {
        public static int INTAKE_TOGGLE = 1;
        public static int INTAKE_REVERSE = 3;
    }
    }
    public static class driverStationPorts {
    static int LEFT_JOY = 0;
    static int RIGHT_JOY = 1;
    static int CLIMBING_DRIVE_BUTTON = 2;
    static int RIGHT_DRIVER_STATION= 2;
    }
}
