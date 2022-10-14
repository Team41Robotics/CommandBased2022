// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.RobotMap.AutonConstants;
import frc.robot.RobotMap.DriverStationConstants;
import frc.robot.RobotMap.DriverStationConstants.LeftJoystick;
import frc.robot.RobotMap.DriverStationConstants.OperatorConsole;
import frc.robot.RobotMap.DriverStationConstants.RightJoystick;
import frc.robot.autonomous.AutonomousRoutine;
import frc.robot.commands.climber.FifthStage;
import frc.robot.commands.climber.FirstStage;
import frc.robot.commands.climber.FourthStage;
import frc.robot.commands.climber.SecondStage;
import frc.robot.commands.climber.ThirdStage;
import frc.robot.commands.drivetrain.DefaultDrive;
import frc.robot.commands.intake.IntakeReverse;
import frc.robot.commands.shooter.AutoShoot;
import frc.robot.commands.shooter.DisableShooter;
import frc.robot.commands.shooter.LineUpNearShot;
import frc.robot.commands.shooter.RunFeeder;
import frc.robot.commands.shooter.SetHoodPosition;
import frc.robot.commands.shooter.ZeroHood;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.Limelight;

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
    public static ClimberSubsystem climber = new ClimberSubsystem();
    public static IntakeSubsystem intake = new IntakeSubsystem();
    public static DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
    public static ShooterSubsystem shooter = new ShooterSubsystem();
    public static HoodSubsystem hood = new HoodSubsystem();

    public static Limelight limelight = new Limelight();

    /* IO */
    public static Joystick rightJoystick = new Joystick(DriverStationConstants.IO.RIGHT_JOYSTICK);
    public static Joystick leftJoystick = new Joystick(DriverStationConstants.IO.LEFT_JOYSTICK);
    public static Joystick operatorConsole = new Joystick(DriverStationConstants.IO.OPERATOR_CONSOLE);

    public static JoystickButton interuptButton = new JoystickButton(operatorConsole, OperatorConsole.INTERUPT_BUTTON);
    public static DigitalInput beamBreak = new DigitalInput(AutonConstants.BEAM_BREAK_PORT);

    /**
     * The command configured to run during auto
     */
    private Command autonomousCommand;

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization co
     */
    @Override
    public void robotInit() {
        drivetrain.setDefaultCommand(new DefaultDrive());

        configureButtonBindings();
        AutonomousRoutine.initShuffleboard();
    }

    /**
     * Runs when autonomous mode is first started
     *
     * We first zero the hood, and then grab the selected auto routine from shuffleboard
     *
     * If an routine is selected, schedule it after a wait command of the configured duration
     */
    @Override
    public void autonomousInit() {
        // Zero the hood

        CommandScheduler.getInstance().schedule(new InstantCommand(intake::putDown));

        // Chooses which auto we do from SmartDashboard
        autonomousCommand = AutonomousRoutine.AUTO_CHOOSER.getSelected().construct();

        // Schedule the selected autonomous command group
        if (autonomousCommand != null) {
            CommandScheduler
                .getInstance()
                .schedule(
                    // To achieve the configured delay, use a sequential group that contains a wait
                    // command
                    new SequentialCommandGroup(
                        new WaitCommand(AutonomousRoutine.AUTO_DELAY_CHOOSER.getSelected()),
                        autonomousCommand
                    )
                );
        }
    }

    /**
     * Runs when teleop starts
     *
     * Zero the hood once more, and hard cancel the auto
     * routine if it is still running for any reason
     */
    @Override
    public void teleopInit() {
        // Zero the hood again when teleop starts
        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(new ZeroHood(), new SetHoodPosition(5)));

        // This makes sure that the autonomous stops running when teleop starts running.
        if (autonomousCommand != null) autonomousCommand.cancel();
    }

    /**
     * Called when test mode starts
     */
    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();

        // Reset all subsystems to take accurate readings
        intake.reset();
        drivetrain.hardSet(0);
        shooter.setSpeed(0);
    }

    private void configureButtonBindings() {
        /* ----- OPERATOR CONSOLE ----- */

        // Perform climb sequence
        new POVButton(operatorConsole, 45, OperatorConsole.CLIMBING_STATE_POV)
        .whenPressed(
                new SequentialCommandGroup(
                    new ZeroHood(),
                    new FirstStage(),
                    new WaitCommand(2.5),
                    new SecondStage(),
                    new WaitCommand(1),
                    new ThirdStage(),
                    new WaitCommand(0.5),
                    new FourthStage(),
                    new WaitCommand(2.5),
                    new FifthStage()
                )
                .until(interuptButton::get)
            );

        // Zero the shooter hood
        new JoystickButton(operatorConsole, OperatorConsole.ZERO_HOOD)
        .whenPressed(new SequentialCommandGroup(new ZeroHood(), new SetHoodPosition(5)));

        // Set hood to 5 FU
        new JoystickButton(operatorConsole, OperatorConsole.SET_HOOD_POS).whenPressed(new SetHoodPosition(5));

        // Line up the robot for a close shot using the limelight, and spin up shooter
        new JoystickButton(operatorConsole, OperatorConsole.LINE_UP_NEAR_SHOT).whenPressed(new LineUpNearShot());

        // Shoot a ball automatically
        new JoystickButton(operatorConsole, OperatorConsole.AUTO_SHOOTING)
        .whenPressed(
                new SequentialCommandGroup(
                    new AutoShoot(),
                    new RunFeeder(true),
                    new WaitCommand(0.75),
                    new RunFeeder(false)
                )
                .until(interuptButton::get)
            );

        /* ----- LEFT JOYSTICK ----- */

        // Toggle intake position when pressed
        new JoystickButton(leftJoystick, LeftJoystick.INTAKE_POSITION_TOGGLE)
        .whenPressed(new InstantCommand(intake::togglePosition, intake));

        // Turn the feeder on when pressed
        new JoystickButton(leftJoystick, LeftJoystick.ENABLE_FEEDER).whenPressed(new RunFeeder(true));

        // Turn the feeder off when pressed
        new JoystickButton(leftJoystick, LeftJoystick.DISABLE_FEEDER).whenPressed(new RunFeeder(false));

        // Turn off shooter when pressed
        new JoystickButton(leftJoystick, LeftJoystick.DISABLE_SHOOTER).whenPressed(new DisableShooter());

        /* ----- RIGHT JOYSTICK ----- */

        // Toggle intake motors when pressed
        new JoystickButton(rightJoystick, RightJoystick.INTAKE_TOGGLE)
        .whenPressed(new InstantCommand(intake::toggleEnabled, intake));

        // While held, spit out balls from the intake
        new JoystickButton(rightJoystick, RightJoystick.INTAKE_REVERSE).whileActiveOnce(new IntakeReverse());
    }

    /*
     * This Robot is configured to run with the WPILib CommandScheduler.
     * ⛔ Nothing should be handled in the below methods ⛔
     */

    @Override
    public void robotPeriodic() {
        /*
         * Runs the Scheduler. This is responsible for polling buttons, adding
         * newly-scheduled
         * commands, running already-scheduled commands, removing finished or
         * interrupted commands,
         * and running subsystem periodic() methods. This must be called from the
         * robot's periodic
         * block in order for anything in the Command-based framework to work.
         */
        CommandScheduler.getInstance().run();
    }
}
