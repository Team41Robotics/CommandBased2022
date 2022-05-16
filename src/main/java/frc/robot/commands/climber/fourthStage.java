// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap.CLIMBER;


/** An example command that uses an example subsystem. */
public class fourthStage extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public fourthStage() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.Climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      ClimberSubsystem.toggleSecondStageRelease();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ClimberSubsystem.setSpeed(-CLIMBER.CLIMBING_SLOW_SPEED);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("ended");
    ClimberSubsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
