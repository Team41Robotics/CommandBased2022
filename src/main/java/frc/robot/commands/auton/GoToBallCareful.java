// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap.Auton;
import frc.robot.RobotMap.drivetrainConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.utils.PhotonCamera;

/** An example command that uses an example subsystem. */
public class GoToBallCareful extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private boolean thirdBallClose = false;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public GoToBallCareful() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

      if (PhotonCamera.getArea() > 5) {
          thirdBallClose = true;
      }
      double angle = PhotonCamera.getYaw();
      double slowSpeed = 0.5;
      DrivetrainSubsystem.runInverseKinematics((drivetrainConstants.BALL_FOLLOWING_kP * -angle),
      !thirdBallClose ? Auton.AUTON_SPEED_M_PER_S : slowSpeed);


    }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DrivetrainSubsystem.setNoRamp(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return DrivetrainSubsystem.getDanger() || !Robot.BeamBreak.get();
  }
}
