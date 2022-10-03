// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap.ShooterConstants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.Limelight;

/** An example command that uses an example subsystem. */
public class AutonomousShootBall extends CommandBase {

  @Override
  public void execute() {
    Robot.drivetrain.setNoRamp(0);
    double distance = Robot.limelight.estimateDistance();
    double speed =
      (distance * ShooterConstants.HOOD_SPEED_SLOPE) +
      ShooterConstants.HOOD_SPEED_OFFSET;
    double angle =
      (distance * distance * ShooterConstants.HOOD_ANGLE_CURVE) +
      (distance * ShooterConstants.HOOD_ANGLE_SLOPE) +
      ShooterConstants.HOOD_ANGLE_OFFSET;
    Limelight.setLedOn(true);
    if (Limelight.targetFound()) {
      ShooterSubsystem.setSpeed(speed / 100);
      HoodSubsystem.setToPosition(angle);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    HoodSubsystem.hoodMotor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return HoodSubsystem.isReady();
  }
}
