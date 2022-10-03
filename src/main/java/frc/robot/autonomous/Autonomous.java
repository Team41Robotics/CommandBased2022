package frc.robot.autonomous;

import static frc.robot.autonomous.AutonomousRoutine.create;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
import frc.robot.commands.shooter.SetHoodPosition;
import frc.robot.commands.shooter.ZeroHood;

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
  static {
    create(
      "Four Ball Auto",
      () ->
        new SequentialCommandGroup(
          new ParallelCommandGroup(
            new SequentialCommandGroup(new AllignToBall(), new GoToBall()),
            new SequentialCommandGroup(new ZeroHood(), new SetHoodPosition(5))
          ),
          new GoalAlign(),
          new PrepareToShoot(),
          new ShootBall().withTimeout(0.5),
          new AllignToBall(),
          new GoToBallCareful(),
          new WaitCommand(1.5),
          new MoveForward(
            AutonConstants.DISTANCE_FROM_HUMAN_STATION,
            -AutonConstants.AUTON_SPEED_M_PER_S
          ),
          new RunCommand(() -> Robot.drivetrain.stop())
          .until(() -> Robot.drivetrain.isReady()),
          new GoalAlign(),
          new PrepareToShoot(),
          new ShootBall().withTimeout(0.5)
        )
    );

	create(
		"Simple Two Ball Auto",
		() ->
		  new SequentialCommandGroup(
			new ParallelCommandGroup(
                new SequentialCommandGroup(    
                    new AllignToBall(),
                    new GoToBall()
            ), 
                new SequentialCommandGroup(
                    new ZeroHood(),
                    new SetHoodPosition(5)
            )
            ),
            new GoalAlign(),
            new PrepareToShoot(),
            new ShootBall().withTimeout(1.5)
		  )
	  );
  }
}
