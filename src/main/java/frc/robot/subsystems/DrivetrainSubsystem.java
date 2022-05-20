package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import frc.robot.Robot;
import frc.robot.RobotMap.Auton;
import frc.robot.RobotMap.driverStation;
import frc.robot.RobotMap.drivetrainConstants;
import frc.robot.RobotMap.limelight;
import frc.robot.RobotMap.driverStation.RightJoy;
import frc.robot.RobotMap.driverStation.SecondDriverStation;
import frc.robot.utils.PID;
import frc.robot.utils.PhotonCamera;
import frc.robot.utils.PositionalPID;
import frc.robot.utils.Limelight;
import frc.robot.Robot;
/** Class for manipulating the robot drivetrain */
public class DrivetrainSubsystem extends SubsystemBase {
    public static long startTime;
    private static boolean climbing;
    private static double angleToBall;
    private static AHRS navx = new AHRS(Port.kUSB);
    private static Joystick leftJoy;
    private static Joystick rightJoy;
    private static PID leftBackPID;
    private static PID leftFrontPID;
    private static PID rightBackPID;
    private static PID rightFrontPID;
    private static PositionalPID ballTrackingPID;
    private static TalonFX talonLF, talonLB, talonRF, talonRB;
    private static TalonFX[] talonList = new TalonFX[4];
    /**
     * Intialize all falcons, their encoders and PID controllers, and joysticks
     */
    public static void initDrivetrain() {
        talonLB = new TalonFX(drivetrainConstants.FALCON_LB);
        talonLF = new TalonFX(drivetrainConstants.FALCON_LF);
        talonRF = new TalonFX(drivetrainConstants.FALCON_RF);
        talonRB = new TalonFX(drivetrainConstants.FALCON_RB);

        talonList[0] = talonLF;
        talonList[1] = talonLB;
        talonList[2] = talonRF;
        talonList[3] = talonRB;

        talonLB.setInverted(true); 
        talonLF.setInverted(true);
        
        leftJoy = Robot.leftJoy;
        rightJoy = Robot.rightJoy;

        leftBackPID = new PID(talonLB, drivetrainConstants.kP, drivetrainConstants.kI, drivetrainConstants.kD, drivetrainConstants.kFF, drivetrainConstants.RAMP_TIME);
        leftFrontPID = new PID(talonLF, drivetrainConstants.kP, drivetrainConstants.kI, drivetrainConstants.kD, drivetrainConstants.kFF, drivetrainConstants.RAMP_TIME);
        rightBackPID = new PID(talonRB, drivetrainConstants.kP, drivetrainConstants.kI, drivetrainConstants.kD, drivetrainConstants.kFF, drivetrainConstants.RAMP_TIME);
        rightFrontPID = new PID(talonRF, drivetrainConstants.kP, drivetrainConstants.kI, drivetrainConstants.kD, drivetrainConstants.kFF, drivetrainConstants.RAMP_TIME);
        
        ballTrackingPID = new PositionalPID(drivetrainConstants.BALLTRACKING_P);
    }

    /**
     * Get position of the robot
     * @return position of the robot in inches
     */
    public static double getPosition() {
        return talonLF.getSelectedSensorPosition()*drivetrainConstants.WHEEL_CONVERSION_FACTOR;
    }

    /**
     * Set position of the robot
     * @param pos position of the robot in inches
     */
    public static void setPosition(double pos) {
        for (TalonFX tal : talonList) {
            tal.setSelectedSensorPosition(pos);
        }
    }

    /**
     * Set the speed of the drivetrain
     * @param speed desired speed of the robot [-1, 1]
     */
    public static void set(double speed) {
        leftBackPID.run(speed);
        leftFrontPID.run(speed);
        rightBackPID.run(speed);
        rightFrontPID.run(speed);
    }

    /**
     * Set the speed of the drivetrain without the ramp in PID
     * @param speed desired speed of the robot [-1, 1]
     */
    public static void setNoRamp(double speed) {
        leftBackPID.runNoRamp(speed);
        leftFrontPID.runNoRamp(speed);
        rightBackPID.runNoRamp(speed);
        rightFrontPID.runNoRamp(speed);
    }

    /**
     * Stop the drivetrain
     */
    public static void stop() {
        set(0);
    }
    
    /**
     * Run the drivetrain in teleoperated mode, with both slow and fast modes, as well as small joystick deadzones
     */
    public static void teleop() { 
        double leftSpeed = joystickTransfer(-leftJoy.getY());
        double rightSpeed = joystickTransfer(-rightJoy.getY());
    
        if(Math.abs(leftSpeed) > (climbing ? drivetrainConstants.JOYSTICK_CLIMBING_MODE_DEADZONE : drivetrainConstants.JOYSTICK_DEADZONE)) {
            setLeft(leftSpeed);
        } else {
            setLeft(0);
        }

        if(Math.abs(rightSpeed) > (climbing ? drivetrainConstants.JOYSTICK_CLIMBING_MODE_DEADZONE : drivetrainConstants.JOYSTICK_DEADZONE)) {
            setRight(rightSpeed);
        } else {
            setRight(0);
        }

        if (rightJoy.getRawButtonPressed(RightJoy.CLIMBING_DRIVE_BUTTON)) {
            climbing = !climbing;
        }
    }

    /**
     * Sets the speed of the left motors
     * @param speed desired speed [-1, 1]
     */
    public static void setLeft(double speed) {
        leftBackPID.run(speed);
        leftFrontPID.run(speed);
    }

    /**
     * Sets the speed of the right motors
     * @param speed desired speed [-1, 1]
     */
    public static void setRight(double speed) {
        rightBackPID.run(speed);
        rightFrontPID.run(speed);
    }

    /**
     * Sets the speed of the left motors without the ramp from PID
     * @param speed desired speed [-1, 1]
     */
    private static void setLeftNoRamp(double speed) {
        leftBackPID.runNoRamp(speed);
        leftFrontPID.runNoRamp(speed);
    }

    /**
     * Sets the speed of the right motors without the ramp from PID
     * @param speed desired speed [-1, 1]
     */
    private static void setRightNoRamp(double speed) {
        rightBackPID.runNoRamp(speed);
        rightFrontPID.runNoRamp(speed);
    }

    /**
     * Adjusts the orientation of the robot in accordance to its relation with the tape
     * @return Whether the robot is within the alignment threshold of the hub
     */
    public static boolean alignToGoal() {
        double angle = Limelight.getRobotAngle();
        if (Limelight.targetFound() && angle>limelight.ALIGNMENT_HORIZONTAL_THRESHHOLD) {
            setRight(-Auton.AUTON_SPEED/2);
            setLeft(Auton.AUTON_SPEED/2);
        } else if (Limelight.targetFound() && angle<-limelight.ALIGNMENT_HORIZONTAL_THRESHHOLD) {
            setRight(Auton.AUTON_SPEED/2);
            setLeft(-Auton.AUTON_SPEED/2);
        } else {
            setNoRamp(0);
            return true;
        }
        return false;
    }

    public static void hardSet(double speed) {
        leftFrontPID.motor.set(ControlMode.PercentOutput, speed);
        leftBackPID.motor.set(ControlMode.PercentOutput, speed);
        rightBackPID.motor.set(ControlMode.PercentOutput, speed);
        rightFrontPID.motor.set(ControlMode.PercentOutput, speed);
    }

    /**
     * Prepare the robot to align to a ball
     */
    public static void setupAlignmentToBall() {
        if (!PhotonCamera.hasTarget()) {
            angleToBall = 0;
        } else {
            angleToBall = PhotonCamera.getYaw();
        }
        navx.zeroYaw();
    }

    /**
     * Align the robot to the nearest ball of the correct color
     * @return Whether the robot is within the alignment threshold of the ball
     */
    public static boolean alignToBall() {
        double angle = navx.getAngle();
        double speed = ballTrackingPID.run(angle);
        if (speed < Auton.AUTON_SPEED/2) speed = Auton.AUTON_SPEED/2;
        if (speed > -Auton.AUTON_SPEED/2) speed = -Auton.AUTON_SPEED/2;
        if (angleToBall - angle>limelight.ALIGNMENT_HORIZONTAL_THRESHHOLD) {
            setRightNoRamp(speed);
            setLeftNoRamp(-speed);
        } else if (angleToBall - angle<-limelight.ALIGNMENT_HORIZONTAL_THRESHHOLD) {
            setRightNoRamp(-speed);
            setLeftNoRamp(speed);
        } else {
            setNoRamp(0);
            return true;
        }
        return false;
    }
    
    /**
     * Run the drivetrain using inverse kinematics
     * @param angularVel the desired angular velocity of the robot (rad/s)
     * @param linearVel the desired linear velocity of the robot (m/s)
     */
    public static void runInverseKinematics(double angularVel, double linearVel) {
        double leftWheelAngularVelocity;
        double rightWheelAngularVelocity;

        rightWheelAngularVelocity = (1/drivetrainConstants.WHEEL_RAD)*(linearVel + angularVel*drivetrainConstants.ROBOT_DIAMETER);
        leftWheelAngularVelocity = (1/drivetrainConstants.WHEEL_RAD)*(linearVel - angularVel*drivetrainConstants.ROBOT_DIAMETER);

        rightWheelAngularVelocity *= drivetrainConstants.WHEEL_RADPERSEC_TO_MOTOR_RPM;
        leftWheelAngularVelocity *= drivetrainConstants.WHEEL_RADPERSEC_TO_MOTOR_RPM;

        setLeft(leftWheelAngularVelocity/drivetrainConstants.MAX_RPM);
        setRight(rightWheelAngularVelocity/drivetrainConstants.MAX_RPM);
    }

    /**
     * Function to convert joystick input to a function (basically joystick acceleration)
     * @param joyVal the inputted value from the joystick
     * @return the adjusted value
     */
    public static double joystickTransfer(double joyVal) {
        if (climbing) {
            double newJoyVal = Math.pow(joyVal, 2);
            newJoyVal *= drivetrainConstants.CLIMBING_DRIVE_MAX_SPEED;
            newJoyVal += drivetrainConstants.CLIMBING_DRIVING_SPEED_OFFSET;
            if (joyVal > 0) {
                return newJoyVal;
            } else {
                return -newJoyVal;
            } 
        } else {
            return joyVal * drivetrainConstants.DRIVETRAIN_MAX_SPEED;
        }
    }

    /**
     * A function to test any features of the drivetrain
     */
    public static void test() {
        if (System.currentTimeMillis() - startTime <= 2000) {
            set(0.1);
        } else {
            set(0);
        }
    }

    /**
     * Output all necessary telemetry data from the drivetrain
     * @param table the base telemetry NetworkTable
     */
    public static void telemetry(NetworkTable table) {
        NetworkTable motorTable = table.getSubTable("motors");

        leftFrontPID.telemetry(motorTable, "Left Front Drivetrain Motor");
        leftBackPID.telemetry(motorTable, "Left Back Drivetrain Motor");
        rightFrontPID.telemetry(motorTable, "Right Front Drivetrain Motor");
        rightBackPID.telemetry(motorTable, "Right Back Drivetrain Motor");
    }

    /**
     * Get whether or not the drivetrain is at the desired speed
     * @return If the drivetrain is at the desired speed
     */
    public static boolean isReady() {
        return leftFrontPID.isReady();
    }

    /**
     * Get if the drivetrain current is too high
     * @return true if any of the drivetrain motors are reading more than 80 amps
     */
    public static boolean getDanger(){
        return (leftBackPID.getCurrent()>80) || (leftFrontPID.getCurrent()>80)|| (rightBackPID.getCurrent()>80) || (rightFrontPID.getCurrent()>80);
    }
}
