// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.climber.*;
import frc.robot.commands.drivetrain.drive;
import frc.robot.commands.intake.intakeReverse;
import frc.robot.RobotMap.driverStation.SecondDriverStation;
import frc.robot.commands.shooter.*;

import frc.robot.RobotMap.*;
import frc.robot.triggers.*;
import frc.robot.RobotMap.driverStation.*;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  /* Subsystems */
  public static ClimberSubsystem Climber = new ClimberSubsystem();
  public static IntakeSubsystem Intake = new IntakeSubsystem();
  public static DrivetrainSubsystem Drivetrain = new DrivetrainSubsystem();
  public static ShooterSubsystem Shooter = new ShooterSubsystem();
  public static HoodSubsystem Hood = new HoodSubsystem();
  /* Buttons */
  public static Joystick leftJoy = new Joystick(driverStationPorts.LEFT_JOY);
  public static Joystick rightJoy = new Joystick(driverStationPorts.RIGHT_JOY);
  public static Joystick secondDS = new Joystick(driverStationPorts.RIGHT_DRIVER_STATION);
  public static JoystickButton interuptButton = new JoystickButton(secondDS, 1);
  public static DigitalInput BeamBreak = new DigitalInput(Auton.BEAM_BREAK_PORT);

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization co
   */
  @Override
  public void robotInit() {
    ClimberSubsystem.initClimber();
    IntakeSubsystem.initIntake();
    DrivetrainSubsystem.initDrivetrain();
    buttonBindings();
    ShooterSubsystem.initShooter();
    HoodSubsystem.initHood();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    new SequentialCommandGroup(new ZeroHood(), new SetHoodPosition(5)).schedule();

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    new SequentialCommandGroup(new ZeroHood(), new SetHoodPosition(5)).schedule();

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    IntakeSubsystem.reset();
    DrivetrainSubsystem.hardSet(0);
    ShooterSubsystem.setSpeed(0);


  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    IntakeSubsystem.reset();
    DrivetrainSubsystem.hardSet(0);
    ShooterSubsystem.setSpeed(0);
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }

  private void buttonBindings() {
    Drivetrain.setDefaultCommand(new drive());
    Intake.setDefaultCommand(new RunCommand(IntakeSubsystem::run, Intake));
    new POVTrigger(45, secondDS, SecondDriverStation.CLIMBING_STATE_POV)
        .whenActive(new SequentialCommandGroup(
            new firstStage(),
            new WaitCommand(2.5),
            new secondStage(),
            new WaitCommand(1),
            new thirdStage(),
            new WaitCommand(0.5),
            new fourthStage(),
            new WaitCommand(2.5),
            new fifthStage()).until(interuptButton::get));

    new JoystickButton(leftJoy, LeftJoy.INTAKE_PISTON_TOGGLE)
        .whenActive(new InstantCommand(IntakeSubsystem::toggle, Intake));
    new JoystickButton(rightJoy, 1)
        .whenActive(new InstantCommand(IntakeSubsystem::toggleMotors, Intake));
    new JoystickButton(rightJoy, RightJoy.INTAKE_REVERSE)
        .whileActiveOnce(new intakeReverse());

    new JoystickButton(secondDS, SecondDriverStation.AUTO_SHOOTING)
    .whenActive(
      new SequentialCommandGroup(
        new AutoShoot(), 
        new PrintCommand("reached"),
        new RunFeeder(true), 
        new WaitCommand(0.75), 
        new RunFeeder(false)
        ).until(interuptButton::get)
      );
    new JoystickButton(secondDS, 2).whenActive(new SequentialCommandGroup(new ZeroHood(), new SetHoodPosition(5)));
    new JoystickButton(secondDS, 3).whenActive(new SetHoodPosition(5));
    new JoystickButton(secondDS, 7).whenActive(new RunFeeder(true));
    new JoystickButton(secondDS, 8).whenActive(new RunFeeder(false));

    /*
     * new POVTrigger(45, secondDS, SecondDriverStation.CLIMBING_STATE_POV
     * .whenActive(new firstStage().until(interuptButton::get));
     * new POVTrigger(90, secondDS, SecondDriverStation.CLIMBING_STATE_POV)
     * .whenActive(new secondStage().until(interuptButton::get));
     */
  }
}
