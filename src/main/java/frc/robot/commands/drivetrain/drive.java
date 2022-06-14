
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import frc.robot.Robot;
import frc.robot.RobotMap.drivetrainConstants;
import frc.robot.RobotMap.driverStation.RightJoy;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Joystick;

/** An example command that uses an example subsystem. */
public class drive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private boolean climbing;
    private Joystick leftJoy = Robot.leftJoy;
    private Joystick rightJoy = Robot.rightJoy;
    private DrivetrainSubsystem drivetrain = Robot.Drivetrain;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public drive() {
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftSpeed = DrivetrainSubsystem.joystickTransfer(-leftJoy.getY());
    double rightSpeed = DrivetrainSubsystem.joystickTransfer(-rightJoy.getY());
    double speedOffset = ((Robot.secondDS.getRawAxis(0)*-1)+1)/2;
    System.out.println(speedOffset);
    if(Math.abs(leftSpeed) > (climbing ? drivetrainConstants.JOYSTICK_CLIMBING_MODE_DEADZONE : drivetrainConstants.JOYSTICK_DEADZONE)) {
        DrivetrainSubsystem.setLeft(leftSpeed*speedOffset);
    } else {
        DrivetrainSubsystem.setLeft(0);
    }
    
    if(Math.abs(rightSpeed) > (climbing ? drivetrainConstants.JOYSTICK_CLIMBING_MODE_DEADZONE : drivetrainConstants.JOYSTICK_DEADZONE)) {
        DrivetrainSubsystem.setRight(rightSpeed*speedOffset);
    } else {
        DrivetrainSubsystem.setRight(0);
    }
    
    if (rightJoy.getRawButtonPressed(RightJoy.CLIMBING_DRIVE_BUTTON)) {
        climbing = !climbing;
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
