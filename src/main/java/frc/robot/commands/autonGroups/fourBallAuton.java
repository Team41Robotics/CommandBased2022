package frc.robot.commands.autonGroups;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotMap.Auton;
import frc.robot.commands.auton.*;
import frc.robot.commands.drivetrain.MoveForward;
import frc.robot.commands.shooter.*;
import frc.robot.subsystems.DrivetrainSubsystem;

public class fourBallAuton extends SequentialCommandGroup {
    /*
    * Goes back to one ball, grabs it, and shoots both balls
    */
    public fourBallAuton() {
        addCommands(
            new ParallelCommandGroup(
                new SequentialCommandGroup(    
                    new AllignToBall(),
                    new goToBall()
            ), 
                new SequentialCommandGroup(
                    new ZeroHood(),
                    new SetHoodPosition(5)
            )
            ),
            new GoalAlign(),
            new AutonShoot(),
            new Shoot().withTimeout(0.5),
            new AllignToBall(),
            new GoToBallCareful(),
            new WaitCommand(1.5),
            new MoveForward(Auton.DISTANCE_FROM_HUMAN_STATION, -Auton.AUTON_SPEED_M_PER_S),
            new RunCommand(() -> DrivetrainSubsystem.stop()).until(()->DrivetrainSubsystem.isReady()),
            new GoalAlign(),
            new AutonShoot(),
            new Shoot().withTimeout(0.5)
        );
    }
    
}
