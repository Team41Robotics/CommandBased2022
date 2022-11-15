package frc.robot.autonomous;

import static frc.robot.autonomous.AutonomousRoutine.create;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.RobotMap.AutonConstants;
import frc.robot.autonomous.commands.AllignToBall;
import frc.robot.autonomous.commands.PrepareToShoot;
import frc.robot.autonomous.commands.GoToBall;
import frc.robot.autonomous.commands.GoToBallCareful;
import frc.robot.autonomous.commands.GoalAlign;
import frc.robot.autonomous.commands.ShootBall;
import frc.robot.commands.drivetrain.MoveForward;
import frc.robot.commands.shooter.AutoShoot;
import frc.robot.commands.shooter.RunFeeder;
import frc.robot.commands.shooter.SetHoodPosition;
import frc.robot.commands.shooter.ZeroHood;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Quick guide to Comand Groups:
 *
 * SequentialComandGroup:
 * Will run all comands in order within it's parentheses
 * Note: If a comand does not have a isFinshed statment the code will be stuck
 * on that command forever
 *
 * ParallelCommandGroup:
 * Will run commands in parallel if they use diffrent SubSystems
 * Note: Both commands will have to finish to move on
 *
 * ParallelRaceGoup:
 * Will run commands in parallel if they use diffrent SubSystems
 * Note: As soon as one command runs it's isfinished method runs then both
 * commands will end
 *
 * ParallelDeadlineGroup
 * Will run commands in parallel if they use diffrent SubSystems
 * Note: Only the first command will finish the group
 */
public class Autonomous {
  static IntakeSubsystem intake = Robot.intake;
  static DrivetrainSubsystem drive = Robot.drivetrain;

  static public void initAutos() {
    create(
        "Four Ball Auto",
        () -> new SequentialCommandGroup(
            new ParallelCommandGroup(
                new SequentialCommandGroup(new AllignToBall(), new GoToBall()),
                new SequentialCommandGroup(new ZeroHood(), new SetHoodPosition(5)),
                new InstantCommand(intake::toggleEnabled, intake)),
            new AutoShoot(),
            new RunFeeder(true),
            new WaitCommand(2),
            new RunFeeder(false),
            new AllignToBall(),
            new GoToBallCareful(),
            new WaitCommand(1.5),
            new MoveForward(
                AutonConstants.DISTANCE_FROM_HUMAN_STATION,
                AutonConstants.AUTON_SPEED_M_PER_S),
            new RunCommand(() -> Robot.drivetrain.stop())
                .until(() -> Robot.drivetrain.isReady()),
            new GoalAlign(),
            new AutoShoot(),
            new RunFeeder(true),
            new WaitCommand(2),
            new RunFeeder(false)).until(() -> drive.getDanger())
            
            );

    create(
        "Simple Two Ball Auto",
        () -> new SequentialCommandGroup(
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new AllignToBall(),
                    new GoToBall()),
                new SequentialCommandGroup(
                    new ZeroHood(),
                    new SetHoodPosition(5)),
                new InstantCommand(intake::toggleEnabled, intake)

            ).withTimeout(4),
            new AutoShoot(),
            new WaitCommand(0.5),
            new RunFeeder(true),
            new WaitCommand(2),
            new RunFeeder(false)));

    create(
        "Taxi Shoot",
        () -> new SequentialCommandGroup(
            new ParallelCommandGroup(
                new MoveForward(40, 1),
                new SequentialCommandGroup(
                  new ZeroHood(),
                  new SetHoodPosition(5))
            )

        .withTimeout(4),
        new PrepareToShoot(),
        new WaitCommand(0.5),
        new RunFeeder(true),
        new WaitCommand(2),
        new RunFeeder(false)));
  }
}
