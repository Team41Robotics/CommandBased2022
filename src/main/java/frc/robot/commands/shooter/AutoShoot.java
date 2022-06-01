// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import frc.robot.Robot;
import frc.robot.RobotMap.ShooterConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.Limelight;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class AutoShoot extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoShoot() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.Hood, Robot.Drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distance = Limelight.estimateDistance();
    double speed = (distance * ShooterConstants.HOOD_SPEED_SLOPE) + ShooterConstants.HOOD_SPEED_OFFSET;
    double angle = (distance * distance * ShooterConstants.HOOD_ANGLE_CURVE) + (distance * ShooterConstants.HOOD_ANGLE_SLOPE) + ShooterConstants.HOOD_ANGLE_OFFSET;;
    Limelight.setLedOn(true);
    if (Limelight.targetFound()) {
        ShooterSubsystem.setSpeed(speed / 100);
        HoodSubsystem.setToPosition(angle);
        DrivetrainSubsystem.alignToGoal();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("SHOOTER READY");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ShooterSubsystem.isReady();
  }
}
