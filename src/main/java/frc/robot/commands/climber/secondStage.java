// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap.driverStation.SecondDriverStation;
import frc.robot.Robot;
import frc.robot.RobotMap.CLIMBER;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.RobotMap.driverStation.LeftJoy;

/** An example command that uses an example subsystem. */
public class secondStage extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private static Joystick leftJoy = new Joystick(0);


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public secondStage() {
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(Robot.Climber);
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

  ClimberSubsystem.setSpeed(CLIMBER.CLIMBING_SLOW_SPEED);
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
    return ClimberSubsystem.getEncoder()>90;
  }
}
