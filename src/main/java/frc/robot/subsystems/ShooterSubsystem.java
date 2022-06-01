package frc.robot.subsystems;
import frc.robot.Robot;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.PID;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.RobotMap.ShooterConstants;;
/**
 * Class to control the shooter on the robot
 */
public class ShooterSubsystem extends SubsystemBase{
    public static boolean reverseOn;
    private static PID leftFalconPID, rightFalconPID;
    public static double speed;
    public static CANSparkMax feeder, elevator;
    private static Joystick rightDS, rightJoy;
    private static TalonFX leftFalcon, rightFalcon;
    
    /**
     * Intialize all components of the shooter
     */
    public static void initShooter() {
        leftFalcon = new TalonFX(ShooterConstants.SHOOTER_TALON_2);
        rightFalcon = new TalonFX(ShooterConstants.SHOOTER_TALON_1);
        rightFalcon.setInverted(true);
        leftFalcon.setInverted(false);
        // maybe add some D
        leftFalconPID = new PID(leftFalcon, ShooterConstants.SHOOTER_kP, ShooterConstants.SHOOTER_kI, ShooterConstants.SHOOTER_kD, ShooterConstants.SHOOTER_kFF, ShooterConstants.SHOOTER_RAMP_TIME);
        rightFalconPID = new PID(rightFalcon, ShooterConstants.SHOOTER_kP, ShooterConstants.SHOOTER_kI, ShooterConstants.SHOOTER_kD, ShooterConstants.SHOOTER_kFF, ShooterConstants.SHOOTER_RAMP_TIME);
        feeder = new CANSparkMax(ShooterConstants.FEEDER_MOTOR, MotorType.kBrushless);
        elevator = new CANSparkMax(ShooterConstants.ELEVATOR_MOTOR, MotorType.kBrushless);
        speed = 0;
        rightJoy = Robot.rightJoy;
        reverseOn = false;
        rightDS = Robot.secondDS;
    }

    /**
     * Runs the teleop functions of the shooter
     */
    public static void teleop() {


    }

    /**
     * Place to put test code for the shooter
     */
    public static void test() {
        if (rightDS.getRawButtonPressed(11)) {
            speed += (speed < 0.89) ? 0.025 : 0;
        } else if (rightDS.getRawButtonPressed(12)) {
            speed -= (speed > 0.01) ? 0.025 : 0;
        }

        setSpeed(speed);
        if (rightDS.getRawButton(1)) {
            feeder.set(ShooterConstants.FEEDER_FULL_SPEED);
        } else {
            feeder.set(0);
        }
    }

    /**
     * Set a desired shooter speed
     * @param speed the desired speed of the shooter [0, 1]
     */
    public static void setSpeed(double speed) {
        leftFalconPID.run(speed);
        rightFalconPID.run(speed);
        if (speed != 0) {
            elevator.set(ShooterConstants.ELEVATOR_FULL_SPEED);
        } else {
            elevator.set(0);
        }
    }

    /**
     * Check if the shooter has reached the desired speed
     * @return Whether or not the shooter is ready
     */
    public static boolean isReady() {
        return leftFalconPID.isReady();
    }

    /**
     * Run the feeder wheel to the shooter
     * @param on true to turn the motor on, false to turn it off
     */
    public static void runFeeder(boolean on) {
        feeder.set(on ? ShooterConstants.FEEDER_FULL_SPEED : 0);
    }

    /**
     * Run the elevator motor manually
     * @param speed the desired speed of the elevator motor
     */
    public static void runElevator(double speed) {
        elevator.set(speed);
    }

    /**
     * Upload all telemetry data for the shooter's systems
     * @param table the base telemetry NetworkTable
     */
    public static void telemetry(NetworkTable table) {
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
        feederMotorTable.getEntry("raw_input_speed").setDouble(feeder.get());
        feederMotorTable.getEntry("output_speed").setDouble(feeder.getEncoder().getVelocity());
        feederMotorTable.getEntry("position").setDouble(feeder.getEncoder().getPosition());
        feederMotorTable.getEntry("current").setDouble(feeder.getOutputCurrent());
        
        NetworkTable elevatorMotorTable = motorTable.getSubTable("Elevator Motor");
        elevatorMotorTable.getEntry("name").setString("Elevator Motor");
        elevatorMotorTable.getEntry("loop_error").setDouble(-1);
        elevatorMotorTable.getEntry("p").setDouble(-1);
        elevatorMotorTable.getEntry("i").setDouble(-1);
        elevatorMotorTable.getEntry("d").setDouble(-1);
        elevatorMotorTable.getEntry("requested_input_speed").setDouble(-1);
        elevatorMotorTable.getEntry("actual_input_speed").setDouble(-1);
        elevatorMotorTable.getEntry("raw_input_speed").setDouble(elevator.get());
        elevatorMotorTable.getEntry("output_speed").setDouble(elevator.getEncoder().getVelocity());
        elevatorMotorTable.getEntry("position").setDouble(elevator.getEncoder().getPosition());
        elevatorMotorTable.getEntry("current").setDouble(elevator.getOutputCurrent());
    }
}
