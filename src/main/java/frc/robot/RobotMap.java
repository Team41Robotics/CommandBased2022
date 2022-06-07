package frc.robot;

public final class RobotMap {
    public static final class CLIMBER {

        public static double CLIMBING_DRIVE_MAX_SPEED = 0.4;
        public static double CLIMBING_DRIVING_SPEED_OFFSET = 4e-4;
        public static double CLIMBING_MAX_SPEED = 0.70;
        public static double CLIMBING_SPEED_SECOND_STAGE = 0.6;
        public static double CLIMBING_SLOW_SPEED = 0.2;
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
    public static final class INTAKE{
        public static int INTAKE_MOTOR = 11;
        public static int CONVEYOR_MOTOR = 3;

        public static double INTAKE_FULL_SPEED = 0.6;
        public static double CONVEYOR_FULL_SPEED = 0.5;
        public static double FEEDER_FULL_SPEED = 0.5;
        public static double ELEVATOR_FULL_SPEED = -0.25;
        public static int PCM_PORT = 12;
        public static int LEFT_SOL_FWD = 3;
        public static int LEFT_SOL_RV = 12;
        public static int RIGHT_SOL_FWD = 2;
        public static int RIGHT_SOL_RV = 13;

    }
    public static final class drivetrainConstants{
            // Ports for drivetrain Talons
    public static int FALCON_LF = 10;
    public static int FALCON_LB = 9;
    public static int FALCON_RF = 7;
    public static int FALCON_RB = 8;
    public static double DRIVETRAIN_MAX_SPEED = 0.65;
    public static double CLIMBING_DRIVE_MAX_SPEED = 0.4;
    public static double CLIMBING_DRIVING_SPEED_OFFSET = 4e-4;
    public static double CLIMBING_MAX_SPEED = 0.85;
    public static double CLIMBING_SPEED_SECOND_STAGE = 0.6;
    public static double CLIMBING_SLOW_SPEED = 0.4;
    public static double JOYSTICK_CURVE_POWER = 3; // x^3 function
    public static double JOYSTICK_DEADZONE = 0.1;
    public static double JOYSTICK_CLIMBING_MODE_DEADZONE = 0.001;
    public static int CLIMBING_PISTON_TIME_DELAY = 500; // ms
    public static double MAX_RPM = 6380;

    // Measurements for the physical robot
    public static double WHEEL_CONVERSION_FACTOR = 0.000322265628223*Math.PI;
    public static double WHEEL_RAD = 0.0762;
    public static double ROBOT_DIAMETER = 0.6604;
    public static double WHEEL_RADPERSEC_TO_MOTOR_RPM = 3000/(11*Math.PI);

    // P: 0.8   I: 0.02   D: 0.004   FF: 1.5
    public static double kP = 0.8;
    public static double kI = 0.02;
    public static double kD = 0.004;
    public static double kFF = 1.5;
    public static double RAMP_TIME = 0.8;

    public static double BALLTRACKING_P = 0.09;
    public static double BALL_FOLLOWING_kP = 0.01;
 
    public static double THIRD_BALL_SPEED_kP = 2;   
    }
    public static class ShooterConstants{
        // Ports and Constants for Hood
        public static int HOOD_SPARK = 5;
        public static int HOOD_TOP_LIMIT_SWITCH = 8;
        public static int HOOD_BOTTOM_LIMIT_SWITCH = 9;
        public static double HOOD_SPEED = 0.4;
        public static double HOOD_MAX_POS = 50;
        public static double HOOD_MIN_POS = 0;

        // Constants for Hood Auto Alligning
        public static double HOOD_SPEED_SLOPE = 0.0731;
        public static double HOOD_SPEED_OFFSET = 35.2;
        public static double HOOD_SPEED_OFFSET_INCREMENT = 0.5;
        public static double HOOD_ANGLE_SLOPE = 0.231;
        public static double HOOD_ANGLE_OFFSET = 8.96;
        public static double HOOD_ANGLE_CURVE = -0.000437;

        public static double SHOOTER_DEFAULT_SPEED = 0.375;
        public static double HOOD_DEFAULT_ANGLE = 20;

        public static double LOW_GOAL_SPEED = 0.22;
        public static double LOW_GOAL_ANGLE = 20;

        public static int SHOOTER_TALON_1 = 14;
        public static int SHOOTER_TALON_2 = 15;
        public static double SHOOTER_SPEED = 0.55;
        public static int FEEDER_MOTOR = 4;
        public static int ELEVATOR_MOTOR = 6;
        public static double FEEDER_FULL_SPEED = 0.5;
        public static double ELEVATOR_FULL_SPEED = -0.25;
        // 0.5, 0.03, 0.0005, 1.1, 1
        public static double SHOOTER_kP = 0.5;
        public static double SHOOTER_kI = 0.03;
        public static double SHOOTER_kD = 0.0005;
        public static double SHOOTER_kFF = 1.1;
        public static double SHOOTER_RAMP_TIME = 1;
        

 
        public static double PID_MIN_ERR = 0.03;
        public static double PID_ERROR = 0.1;


}

    public static class Auton{
            // Auton parameters
    public static double AUTON_SPEED = 0.1;
    public static int AUTON_DISTANCE = 50;
    public static int BALL_DISTANCE_FROM_BOT = 20;
    public static int AUTON_BALL_AREA_THRESHOLD = 20;
    public static double AUTON_SPEED_M_PER_S = 1.5;
    public static int AUTON_WAIT_LOOPS = 525;
    public static int AUTON_SHOOTER_WAIT_TIME = 1000;
    public static int AUTON_HUMAN_BALL_WAIT_TIME = 1000;
    public static int SIMPLE_AUTON_DISTANCE = 60;
    public static int DISTANCE_FROM_HUMAN_STATION = 75;
    public static int BEAM_BREAK_PORT = 7;

    }
    public static final class limelight{
        // Measurements for the Limelight
        public static double LIMELIGHT_HEIGHT_OF_TARGET = 103.5;
        public static double LIMELIGHT_HEIGHT = 35;
        public static double LIMELIGHT_ANGLE = 24.5;
        public static double ALIGNMENT_HORIZONTAL_THRESHHOLD = 1; 
        public static double DRIFTING_HORIZONTAL_THRESHOLD = 10; 
        public static double LIMELIGHT_DEPTH_OFFSET = 2;
        public static double LIMELIGHT_HORIZONTAL_OFFSET = -4.5;
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
        public static int CLIMBING_DRIVE_BUTTON = 2;
    }
    }
    public static final class driverStationPorts {
    public static int LEFT_JOY = 0;
    public static int RIGHT_JOY = 1;

    public static int RIGHT_DRIVER_STATION= 2;
    }

    public enum INTAKE_MODE {
        FORWARD,
        REVERSE,
    }
}
