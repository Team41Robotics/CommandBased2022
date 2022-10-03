// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;

public class SecondStage extends CommandBase {

    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public SecondStage() {
        // Use addRequirements() here to declare subsystem dependencies.

        addRequirements(Robot.climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        ClimberSubsystem.reset();
        ClimberSubsystem.resetEncoders(0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        ClimberSubsystem.setSpeed(ClimberConstants.CLIMBING_SPEED_SECOND_STAGE);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("ended");
        ClimberSubsystem.setSpeed(0);
        ClimberSubsystem.lockFirstStage(true);
        ClimberSubsystem.lockSecondStage(false);
        ClimberSubsystem.toggleGearShifter();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return ClimberSubsystem.getEncoder() > 90;
    }
}
  public boolean isFinished() {
    return ClimberSubsystem.getEncoder()>90;
  }
}
