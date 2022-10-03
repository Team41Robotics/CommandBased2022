package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.IntakeConstants;

/**
 * Controls the intake on the robot
 */
public class IntakeSubsystem extends SubsystemBase {

    private boolean enabled = false;
    private IntakeDirection direction = IntakeDirection.FORWARD;
    private IntakePosition position = IntakePosition.UP;

    private CANSparkMax intakeMotor, conveyorMotor;
    private DoubleSolenoid intakeLeftSolenoid;
    private DoubleSolenoid intakeRightSolenoid;

    public IntakeSubsystem() {
        intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR, MotorType.kBrushless);
        intakeMotor.setIdleMode(IdleMode.kCoast);

        conveyorMotor = new CANSparkMax(IntakeConstants.CONVEYOR_MOTOR, MotorType.kBrushless);
        conveyorMotor.setInverted(true);

        intakeLeftSolenoid =
            new DoubleSolenoid(
                IntakeConstants.PCM_PORT,
                PneumaticsModuleType.REVPH,
                IntakeConstants.LEFT_SOL_FWD,
                IntakeConstants.LEFT_SOL_RV
            );

        intakeRightSolenoid =
            new DoubleSolenoid(
                IntakeConstants.PCM_PORT,
                PneumaticsModuleType.REVPH,
                IntakeConstants.RIGHT_SOL_FWD,
                IntakeConstants.RIGHT_SOL_RV
            );

        putDown();
    }

    @Override
    public void periodic() {
        double intakeSpeed = enabled ? IntakeConstants.INTAKE_FULL_SPEED : 0;
        double conveyorSpeed = enabled ? IntakeConstants.CONVEYOR_FULL_SPEED : 0;

        intakeSpeed *= direction == IntakeDirection.FORWARD ? 1 : -1;

        intakeMotor.set(intakeSpeed);
        conveyorMotor.set(conveyorSpeed);
    }

    public IntakeDirection getDirection() {
        return direction;
    }

    public void setDirection(IntakeDirection direction) {
        this.direction = direction;
    }

    /**
     * Make the intake solenoids put the intake in the up position
     */
    public void putUp() {
        intakeLeftSolenoid.set(Value.kReverse);
        intakeRightSolenoid.set(Value.kReverse);
        position = IntakePosition.UP;
    }

    /**
     * Make the intake solenoids put the intake in the down position
     */
    public void putDown() {
        intakeLeftSolenoid.set(Value.kForward);
        intakeRightSolenoid.set(Value.kForward);
        position = IntakePosition.DOWN;
    }

    public void togglePosition() {
        if (position == IntakePosition.UP) {
            putDown();
        } else {
            putUp();
        }
    }

	public void enable() {
		enabled = true;
	}

	public void disable() {
		intakeMotor.set(0);
		conveyorMotor.set(0);
		enabled = false;
	}

    public boolean isEnabled() {
        return enabled;
    }

	public void toggleEnabled() {
        if (!enabled) {
            enable();
        } else {
            disable();
        }
    }

    public IntakePosition getPosition() {
        return position;
    }

    /**
     * Reset the intake to a known position and state
     */
    public void reset() {
        putUp();
        disable();
    }

    /**
     * Record all telemetry data for the intake
     * @param table the base telemetry NetworkTable
     */
    public void telemetry(NetworkTable table) {
        NetworkTable motorTable = table.getSubTable("motors");

        NetworkTable intakeMotorTable = motorTable.getSubTable("Intake Motor");
        intakeMotorTable.getEntry("name").setString("Intake Motor");
        intakeMotorTable.getEntry("loop_error").setDouble(-1);
        intakeMotorTable.getEntry("p").setDouble(-1);
        intakeMotorTable.getEntry("i").setDouble(-1);
        intakeMotorTable.getEntry("d").setDouble(-1);
        intakeMotorTable.getEntry("requested_input_speed").setDouble(-1);
        intakeMotorTable.getEntry("actual_input_speed").setDouble(-1);
        intakeMotorTable.getEntry("raw_input_speed").setDouble(intakeMotor.get());
        intakeMotorTable.getEntry("output_speed").setDouble(intakeMotor.getEncoder().getVelocity());
        intakeMotorTable.getEntry("position").setDouble(intakeMotor.getEncoder().getPosition());
        intakeMotorTable.getEntry("current").setDouble(intakeMotor.getOutputCurrent());

        NetworkTable conveyorMotorTable = motorTable.getSubTable("Conveyor Motor");
        conveyorMotorTable.getEntry("name").setString("Conveyor Motor");
        conveyorMotorTable.getEntry("loop_error").setDouble(-1);
        conveyorMotorTable.getEntry("p").setDouble(-1);
        conveyorMotorTable.getEntry("i").setDouble(-1);
        conveyorMotorTable.getEntry("d").setDouble(-1);
        conveyorMotorTable.getEntry("requested_input_speed").setDouble(-1);
        conveyorMotorTable.getEntry("actual_input_speed").setDouble(-1);
        conveyorMotorTable.getEntry("raw_input_speed").setDouble(conveyorMotor.get());
        conveyorMotorTable.getEntry("output_speed").setDouble(conveyorMotor.getEncoder().getVelocity());
        conveyorMotorTable.getEntry("position").setDouble(conveyorMotor.getEncoder().getPosition());
        conveyorMotorTable.getEntry("current").setDouble(conveyorMotor.getOutputCurrent());

        NetworkTable solenoidTable = table.getSubTable("solenoids");

        solenoidTable.getEntry("intake_position").setString(position.toString());

        NetworkTable intakeSolLeftTable = solenoidTable.getSubTable("Left Intake Solenoid");
        intakeSolLeftTable.getEntry("name").setString("Left Intake Solenoid");
        intakeSolLeftTable
            .getEntry("status")
            .setDouble(
                intakeLeftSolenoid.get() == Value.kForward ? 1 : (intakeLeftSolenoid.get() == Value.kReverse ? -1 : 0)
            );

        NetworkTable intakeSolRightTable = solenoidTable.getSubTable("Right Intake Solenoid");
        intakeSolRightTable.getEntry("name").setString("Right Intake Solenoid");
        intakeSolRightTable
            .getEntry("status")
            .setDouble(
                intakeRightSolenoid.get() == Value.kForward ? 1 : (intakeRightSolenoid.get() == Value.kReverse ? -1 : 0)
            );
    }

    public enum IntakeDirection {
        FORWARD,
        REVERSE,
    }

    public enum IntakePosition {
        UP,
        DOWN,
    }
}
