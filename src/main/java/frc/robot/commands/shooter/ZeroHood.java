// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import frc.robot.Robot;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.RobotMap.ShooterConstants;
import edu.wpi.first.wpilibj2.command.CommandBase;


/** An example command that uses an example subsystem. */
public class ZeroHood extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ZeroHood() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.Hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    HoodSubsystem.hoodMotor.set(-ShooterConstants.HOOD_SPEED/4);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    HoodSubsystem.enc.setPosition(0);
    HoodSubsystem.hoodMotor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !HoodSubsystem.getBottomSwitch();
  }
}
