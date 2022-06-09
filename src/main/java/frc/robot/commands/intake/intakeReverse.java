package frc.robot.commands.intake;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap.INTAKE;
import frc.robot.RobotMap.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;


/** An example command that uses an example subsystem. */
public class intakeReverse extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public intakeReverse() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.Shooter, Robot.Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ShooterSubsystem.elevator.set(-ShooterConstants.ELEVATOR_FULL_SPEED);
    ShooterSubsystem.feeder.set(-ShooterConstants.FEEDER_FULL_SPEED);
    IntakeSubsystem.intakeMotor.set(-INTAKE.INTAKE_FULL_SPEED);
    IntakeSubsystem.conveyor.set(-INTAKE.CONVEYOR_FULL_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ShooterSubsystem.elevator.set(0);
    ShooterSubsystem.feeder.set(0);
    IntakeSubsystem.intakeMotor.set(0);
    IntakeSubsystem.conveyor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return false;
  }
}