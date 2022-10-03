package frc.robot.commands.autonGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.commands.AllignToBall;
import frc.robot.autonomous.commands.AutonomousShootBall;
import frc.robot.autonomous.commands.GoToBall;
import frc.robot.autonomous.commands.GoalAlign;
import frc.robot.autonomous.commands.Shoot;
import frc.robot.commands.auton.*;
import frc.robot.commands.shooter.*;

public class SimpleTwoBall extends SequentialCommandGroup {
    /*
    * Goes back to one ball, grabs it, and shoots both balls
    */
    public SimpleTwoBall() {
        addCommands(
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
            new AutonomousShootBall(),
            new Shoot().withTimeout(1.5)
        );
    }
    
}
