package frc.robot.commands.autonGroups;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotMap.AutonConstants;
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
            
        );
    }
    
}
