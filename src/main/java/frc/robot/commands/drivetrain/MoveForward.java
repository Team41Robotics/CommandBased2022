// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

/** An example command that uses an example subsystem. */
public class MoveForward extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private double startPos;
  private double endPos;
  private double MovementSpeed;
  /**
   * Creates a new ExampleCommand.
   *
   * @param distnce The subsystem used by this command.
   */
  public MoveForward(double distance, double speed) {
    endPos = distance;
    MovementSpeed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startPos=DrivetrainSubsystem.getPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DrivetrainSubsystem.runInverseKinematics(0.075, MovementSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return abs(startPos-DrivetrainSubsystem.getPosition()) - endPos < 1;
  }
}
