// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;
import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap.Auton;
import frc.robot.RobotMap.ShooterConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.Limelight;

/** An example command that uses an example subsystem. */
public class goToBallNoVis extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public goToBallNoVis() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ShooterSubsystem.setSpeed(ShooterConstants.SHOOTER_DEFAULT_SPEED);
    Limelight.setLedOn(true);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Beam break here

    DrivetrainSubsystem.runInverseKinematics(0, Auton.AUTON_SPEED_M_PER_S * (2.0 / 3));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    do {
      DrivetrainSubsystem.setPosition(0);
  } while (DrivetrainSubsystem.getPosition() > 1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !Robot.BeamBreak.get();
  }
}
