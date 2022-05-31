package frc.robot.commands.intake;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap.INTAKE_MODE;


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
    addRequirements(Robot.Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    IntakeSubsystem.direction = INTAKE_MODE.REVERSE;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("ended");
    IntakeSubsystem.direction = INTAKE_MODE.FORWARD;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return false;
  }
}