// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

/** An example command that uses an example subsystem. */
public class AllignToBall extends CommandBase {
  private boolean finished = false;

  @Override
  public void execute() {
    this.finished = Robot.drivetrain.alignToBall();
  }

  @Override
  public void end(boolean interrupted) {
    Robot.drivetrain.setNoRamp(0);
  }

  @Override
  public boolean isFinished() {
    return this.finished;
  }
}
