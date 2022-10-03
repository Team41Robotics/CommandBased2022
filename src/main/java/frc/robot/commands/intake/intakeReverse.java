package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem.IntakeDirection;

/** An example command that uses an example subsystem. */
public class IntakeReverse extends CommandBase {

    public IntakeReverse() {
        addRequirements(Robot.shooter, Robot.intake);
    }

    @Override
    public void initialize() {
        Robot.intake.setDirection(IntakeDirection.REVERSE);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Robot.shooter.elevatorMotor.set(-ShooterConstants.ELEVATOR_FULL_SPEED);
        Robot.shooter.feederMotor.set(-ShooterConstants.FEEDER_FULL_SPEED);
        Robot.intake.enable();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.shooter.elevatorMotor.set(0);
        Robot.shooter.feederMotor.set(0);

        Robot.intake.setDirection(IntakeDirection.FORWARD);
        Robot.intake.disable();
    }
}
