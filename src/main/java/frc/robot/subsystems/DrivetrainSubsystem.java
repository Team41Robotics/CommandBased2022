package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap.AutonConstants;
import frc.robot.RobotMap.DriverStationConstants.RightJoystick;
import frc.robot.RobotMap.DrivetrainConstants;
import frc.robot.RobotMap.LimelightConstants;
import frc.robot.utils.PID;
import frc.robot.utils.PhotonCamera;
import frc.robot.utils.PositionalPID;

/** Class for manipulating the robot drivetrain */
public class DrivetrainSubsystem extends SubsystemBase {

  public long startTime;
  private double angleToBall;
  private AHRS navx = new AHRS(Port.kUSB);
  private Joystick leftJoy;
  private Joystick rightJoy;
  private PID leftBackPID;
  private PID leftFrontPID;
  private PID rightBackPID;
  private PID rightFrontPID;
  private PositionalPID ballTrackingPID;
  private TalonFX talonLF, talonLB, talonRF, talonRB;
  private TalonFX[] talonList = new TalonFX[4];

  public DrivetrainSubsystem() {
    talonLB = new TalonFX(DrivetrainConstants.FALCON_LB);
    talonLF = new TalonFX(DrivetrainConstants.FALCON_LF);
    talonRF = new TalonFX(DrivetrainConstants.FALCON_RF);
    talonRB = new TalonFX(DrivetrainConstants.FALCON_RB);

    talonList[0] = talonLF;
    talonList[1] = talonLB;
    talonList[2] = talonRF;
    talonList[3] = talonRB;

    talonLB.setInverted(true);
    talonLF.setInverted(true);

    leftBackPID =
      new PID(
        talonLB,
        DrivetrainConstants.kP,
        DrivetrainConstants.kI,
        DrivetrainConstants.kD,
        DrivetrainConstants.kFF,
        DrivetrainConstants.RAMP_TIME
      );
    leftFrontPID =
      new PID(
        talonLF,
        DrivetrainConstants.kP,
        DrivetrainConstants.kI,
        DrivetrainConstants.kD,
        DrivetrainConstants.kFF,
        DrivetrainConstants.RAMP_TIME
      );
    rightBackPID =
      new PID(
        talonRB,
        DrivetrainConstants.kP,
        DrivetrainConstants.kI,
        DrivetrainConstants.kD,
        DrivetrainConstants.kFF,
        DrivetrainConstants.RAMP_TIME
      );
    rightFrontPID =
      new PID(
        talonRF,
        DrivetrainConstants.kP,
        DrivetrainConstants.kI,
        DrivetrainConstants.kD,
        DrivetrainConstants.kFF,
        DrivetrainConstants.RAMP_TIME
      );

    ballTrackingPID = new PositionalPID(DrivetrainConstants.BALLTRACKING_P);
  }

  /**
   * Get position of the robot
   * @return position of the robot in inches
   */
  public double getPosition() {
    return (
      talonLF.getSelectedSensorPosition() *
      DrivetrainConstants.WHEEL_CONVERSION_FACTOR
    );
  }

  /**
   * Set position of the robot
   * @param pos position of the robot in inches
   */
  public void setPosition(double pos) {
    for (TalonFX tal : talonList) {
      tal.setSelectedSensorPosition(pos);
    }
  }

  /**
   * Set the speed of the drivetrain
   * @param speed desired speed of the robot [-1, 1]
   */
  public void set(double speed) {
    leftBackPID.run(speed);
    leftFrontPID.run(speed);
    rightBackPID.run(speed);
    rightFrontPID.run(speed);
  }

  /**
   * Set the speed of the drivetrain without the ramp in PID
   * @param speed desired speed of the robot [-1, 1]
   */
  public void setNoRamp(double speed) {
    leftBackPID.runNoRamp(speed);
    leftFrontPID.runNoRamp(speed);
    rightBackPID.runNoRamp(speed);
    rightFrontPID.runNoRamp(speed);
  }

  /**
   * Stop the drivetrain
   */
  public void stop() {
    set(0);
  }

  /**
   * Sets the speed of the left motors
   * @param speed desired speed [-1, 1]
   */
  public void setLeft(double speed) {
    leftBackPID.run(speed);
    leftFrontPID.run(speed);
  }

  /**
   * Sets the speed of the right motors
   * @param speed desired speed [-1, 1]
   */
  public void setRight(double speed) {
    rightBackPID.run(speed);
    rightFrontPID.run(speed);
  }

  /**
   * Sets the speed of the left motors without the ramp from PID
   * @param speed desired speed [-1, 1]
   */
  private void setLeftNoRamp(double speed) {
    leftBackPID.runNoRamp(speed);
    leftFrontPID.runNoRamp(speed);
  }

  /**
   * Sets the speed of the right motors without the ramp from PID
   * @param speed desired speed [-1, 1]
   */
  private void setRightNoRamp(double speed) {
    rightBackPID.runNoRamp(speed);
    rightFrontPID.runNoRamp(speed);
  }

  /**
   * Adjusts the orientation of the robot in accordance to its relation with the tape
   * @return Whether the robot is within the alignment threshold of the hub
   */
  public boolean alignToGoal() {
    double angle = Robot.limelight.getRobotAngle();
    if (
      Robot.limelight.targetFound() &&
      angle > LimelightConstants.ALIGNMENT_HORIZONTAL_THRESHHOLD
    ) {
      setRight(-AutonConstants.AUTON_SPEED / 2);
      setLeft(AutonConstants.AUTON_SPEED / 2);
    } else if (
      Robot.limelight.targetFound() &&
      angle < -LimelightConstants.ALIGNMENT_HORIZONTAL_THRESHHOLD
    ) {
      setRight(AutonConstants.AUTON_SPEED / 2);
      setLeft(-AutonConstants.AUTON_SPEED / 2);
    } else {
      setNoRamp(0);
      return true;
    }
    return false;
  }

  public void hardSet(double speed) {
    leftFrontPID.motor.set(ControlMode.PercentOutput, speed);
    leftBackPID.motor.set(ControlMode.PercentOutput, speed);
    rightBackPID.motor.set(ControlMode.PercentOutput, speed);
    rightFrontPID.motor.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Prepare the robot to align to a ball
   */
  public void setupAlignmentToBall() {
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
  public boolean alignToBall() {
    double angle = navx.getAngle();
    double speed = ballTrackingPID.run(angle);
    if (speed < AutonConstants.AUTON_SPEED / 2) speed =
      AutonConstants.AUTON_SPEED / 2;
    if (speed > -AutonConstants.AUTON_SPEED / 2) speed =
      -AutonConstants.AUTON_SPEED / 2;
    if (
      angleToBall - angle > LimelightConstants.ALIGNMENT_HORIZONTAL_THRESHHOLD
    ) {
      setRightNoRamp(speed);
      setLeftNoRamp(-speed);
    } else if (
      angleToBall - angle < -LimelightConstants.ALIGNMENT_HORIZONTAL_THRESHHOLD
    ) {
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
  public void runInverseKinematics(double angularVel, double linearVel) {
    double leftWheelAngularVelocity;
    double rightWheelAngularVelocity;

    rightWheelAngularVelocity =
      (1 / DrivetrainConstants.WHEEL_RAD) *
      (linearVel + angularVel * DrivetrainConstants.ROBOT_DIAMETER);
    leftWheelAngularVelocity =
      (1 / DrivetrainConstants.WHEEL_RAD) *
      (linearVel - angularVel * DrivetrainConstants.ROBOT_DIAMETER);

    rightWheelAngularVelocity *=
      DrivetrainConstants.WHEEL_RADPERSEC_TO_MOTOR_RPM;
    leftWheelAngularVelocity *=
      DrivetrainConstants.WHEEL_RADPERSEC_TO_MOTOR_RPM;

    setLeft(leftWheelAngularVelocity / DrivetrainConstants.MAX_RPM);
    setRight(rightWheelAngularVelocity / DrivetrainConstants.MAX_RPM);
  }

  /**
   * A function to test any features of the drivetrain
   */
  public void test() {
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
  public void telemetry(NetworkTable table) {
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
  public boolean isReady() {
    return leftFrontPID.isReady();
  }

  /**
   * Get if the drivetrain current is too high
   * @return true if any of the drivetrain motors are reading more than 80 amps
   */
  public boolean getDanger() {
    return (
      (leftBackPID.getCurrent() > 80) ||
      (leftFrontPID.getCurrent() > 80) ||
      (rightBackPID.getCurrent() > 80) ||
      (rightFrontPID.getCurrent() > 80)
    );
  }

  public void drive(double left, double right) {
	  setLeft(left);
	  setLeft(right);
  }
}
