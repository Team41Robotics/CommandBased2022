package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.ShooterConstants;
import frc.robot.utils.PID;

/**
 * Class to control the shooter on the robot
 */
public class ShooterSubsystem extends SubsystemBase {

    public CANSparkMax feederMotor, elevatorMotor;

    private PID leftFalconPID, rightFalconPID;
    private TalonFX leftFalcon, rightFalcon;

    /**
     * Intialize all components of the shooter
     */
    public ShooterSubsystem() {
        rightFalcon = new TalonFX(ShooterConstants.SHOOTER_TALON_1);
        rightFalcon.setInverted(true);

        leftFalcon = new TalonFX(ShooterConstants.SHOOTER_TALON_2);
        leftFalcon.setInverted(false);

        leftFalconPID =
            new PID(
                leftFalcon,
                ShooterConstants.SHOOTER_kP,
                ShooterConstants.SHOOTER_kI,
                ShooterConstants.SHOOTER_kD,
                ShooterConstants.SHOOTER_kFF,
                ShooterConstants.SHOOTER_RAMP_TIME
            );
        rightFalconPID =
            new PID(
                rightFalcon,
                ShooterConstants.SHOOTER_kP,
                ShooterConstants.SHOOTER_kI,
                ShooterConstants.SHOOTER_kD,
                ShooterConstants.SHOOTER_kFF,
                ShooterConstants.SHOOTER_RAMP_TIME
            );

        feederMotor = new CANSparkMax(ShooterConstants.FEEDER_MOTOR, MotorType.kBrushless);
        elevatorMotor = new CANSparkMax(ShooterConstants.ELEVATOR_MOTOR, MotorType.kBrushless);

        elevatorMotor.set(0);
        feederMotor.set(0);

        setSpeed(0);
    }

    /**
     * Set a desired shooter speed
     * @param speed the desired speed of the shooter [0, 1]
     */
    public void setSpeed(double speed) {
        leftFalconPID.run(speed);
        rightFalconPID.run(speed);

		// If shooter is running, run the elevator too
        if (speed == 0) {
            elevatorMotor.set(0);
        } else {
            elevatorMotor.set(ShooterConstants.ELEVATOR_FULL_SPEED);
        }
    }

    /**
     * Check if the shooter has reached the desired speed
     * @return Whether or not the shooter is ready
     */
    public boolean isReady() {
        return leftFalconPID.isReady();
    }

    /**
     * Run the feeder wheel to the shooter
     * @param on true to turn the motor on, false to turn it off
     */
    public void runFeeder(boolean on) {
        feederMotor.set(on ? ShooterConstants.FEEDER_FULL_SPEED : 0);
    }

    /**
     * Run the elevator motor manually
     * @param speed the desired speed of the elevator motor
     */
    public void runElevator(double speed) {
        elevatorMotor.set(speed);
    }

    /**
     * Upload all telemetry data for the shooter's systems
     * @param table the base telemetry NetworkTable
     */
    public void telemetry(NetworkTable table) {
        NetworkTable motorTable = table.getSubTable("motors");

        leftFalconPID.telemetry(motorTable, "Left Shooter Motor");
        rightFalconPID.telemetry(motorTable, "Right Shooter Motor");

        NetworkTable feederMotorTable = motorTable.getSubTable("Feeder Motor");
        feederMotorTable.getEntry("name").setString("Feeder Motor");
        feederMotorTable.getEntry("loop_error").setDouble(-1);
        feederMotorTable.getEntry("p").setDouble(-1);
        feederMotorTable.getEntry("i").setDouble(-1);
        feederMotorTable.getEntry("d").setDouble(-1);
        feederMotorTable.getEntry("requested_input_speed").setDouble(-1);
        feederMotorTable.getEntry("actual_input_speed").setDouble(-1);
        feederMotorTable.getEntry("raw_input_speed").setDouble(feederMotor.get());
        feederMotorTable.getEntry("output_speed").setDouble(feederMotor.getEncoder().getVelocity());
        feederMotorTable.getEntry("position").setDouble(feederMotor.getEncoder().getPosition());
        feederMotorTable.getEntry("current").setDouble(feederMotor.getOutputCurrent());

        NetworkTable elevatorMotorTable = motorTable.getSubTable("Elevator Motor");
        elevatorMotorTable.getEntry("name").setString("Elevator Motor");
        elevatorMotorTable.getEntry("loop_error").setDouble(-1);
        elevatorMotorTable.getEntry("p").setDouble(-1);
        elevatorMotorTable.getEntry("i").setDouble(-1);
        elevatorMotorTable.getEntry("d").setDouble(-1);
        elevatorMotorTable.getEntry("requested_input_speed").setDouble(-1);
        elevatorMotorTable.getEntry("actual_input_speed").setDouble(-1);
        elevatorMotorTable.getEntry("raw_input_speed").setDouble(elevatorMotor.get());
        elevatorMotorTable.getEntry("output_speed").setDouble(elevatorMotor.getEncoder().getVelocity());
        elevatorMotorTable.getEntry("position").setDouble(elevatorMotor.getEncoder().getPosition());
        elevatorMotorTable.getEntry("current").setDouble(elevatorMotor.getOutputCurrent());
    }
}
