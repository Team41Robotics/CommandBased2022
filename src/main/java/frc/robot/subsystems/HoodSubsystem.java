package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap.ShooterConstants;

public class HoodSubsystem extends SubsystemBase{
    public static boolean ready, homed;
    public static double angle;
    public static CANSparkMax hoodMotor;
    public static DigitalInput topSwitch, bottomSwitch;
    private static Joystick station;
    public static RelativeEncoder enc;

    /**
     * Initialize all motors, encoders, and switches needed to operate the hood
     */
    public static void initHood() {
        // Bottom switch needs to be inverted
        hoodMotor = new CANSparkMax(ShooterConstants.HOOD_SPARK, MotorType.kBrushless);
        topSwitch = new DigitalInput(ShooterConstants.HOOD_TOP_LIMIT_SWITCH);
        bottomSwitch = new DigitalInput(ShooterConstants.HOOD_BOTTOM_LIMIT_SWITCH);
        hoodMotor.setIdleMode(IdleMode.kBrake);
        enc = hoodMotor.getEncoder();
        enc.setPosition(0);
        station = Robot.secondDS;
        angle = 0;
        ready = false;
        homed = false;
    }

    /**
     * Function to control the hood's homing during teleop
     */
    public static void teleop() {
        if (!homed) {
            home();
        } else {
            hoodMotor.set(0);
        }
        if(station.getPOV(0)==90){
            home();
        }
    }

    /**
     * Sets the hood to a specified position
     * @param angle the desired angle of the hood (in rotations of the motor)
     */
    public static void setToPosition(double angle) {
        double pos = enc.getPosition();
        if (topSwitch.get() && (pos - angle) < -1 && pos < ShooterConstants.HOOD_MAX_POS) {
            hoodMotor.set(ShooterConstants.HOOD_SPEED/2);
            ready = false;
        } else if (bottomSwitch.get() && (pos-angle) > 1 && pos > ShooterConstants.HOOD_MIN_POS) {
            hoodMotor.set(-ShooterConstants.HOOD_SPEED/4);
            ready = false;
        } else {
            hoodMotor.set(0);
            ready = true;
        }
    }

    /**
     * A place to add testing code for the hood
     */
    public static void test() {
        double pos = enc.getPosition();
        if (topSwitch.get() && station.getRawButtonPressed(9) && pos <= ShooterConstants.HOOD_MAX_POS) {
            angle += (angle < ShooterConstants.HOOD_MAX_POS) ? 1 : 0;
        } else if (bottomSwitch.get() && station.getRawButtonPressed(10)) {
            angle -= (angle > ShooterConstants.HOOD_MIN_POS) ? 1 : 0;
        }

        setToPosition(angle);
    }

    /**
     * Homes the hood asynchronously (needs to be called more than once to finish)
     */
    public static void home() {
        if (bottomSwitch.get()) {
            hoodMotor.set(-ShooterConstants.HOOD_SPEED/4);
        } else {
            enc.setPosition(0);
            hoodMotor.set(0);
            homed = true;
        }
    }

    /**
     * Get if the hood is homed
     * @return true if the hood is homed, false if not
     */
    public static boolean isHomed() {
        return homed;
    }

    /**
     * Get the ready status of the hood
     * @return Whether the hood is in the desired position
     */
    public static boolean isReady() {
        return ready;
    }

    /**
     * Get the value of the hood limit switch at the bottom of its motion
     * @return the value of the switch
     */
    public static boolean getBottomSwitch() {
        return bottomSwitch.get();
    }

    /**
     * Get the value of the hood limit switch at the top of its motion
     * @return the value of the switch
     */
    public static boolean getTopSwitch() {
        return topSwitch.get();
    }

    /**
     * Upload all telemetry data for the hood
     * @param table the base telemetry NetworkTable
     */
    public static void telemetry(NetworkTable table) {
        NetworkTable motorTable = table.getSubTable("motors");
        
        NetworkTable hoodMotorTable = motorTable.getSubTable("Hood Motor");
        hoodMotorTable.getEntry("name").setString("Hood Motor");
        hoodMotorTable.getEntry("loop_error").setDouble(-1);
        hoodMotorTable.getEntry("p").setDouble(-1);
        hoodMotorTable.getEntry("i").setDouble(-1);
        hoodMotorTable.getEntry("d").setDouble(-1);
        hoodMotorTable.getEntry("requested_input_speed").setDouble(-1);
        hoodMotorTable.getEntry("actual_input_speed").setDouble(-1);
        hoodMotorTable.getEntry("raw_input_speed").setDouble(hoodMotor.get());
        hoodMotorTable.getEntry("output_speed").setDouble(hoodMotor.getEncoder().getVelocity());
        hoodMotorTable.getEntry("position").setDouble(hoodMotor.getEncoder().getPosition());
        hoodMotorTable.getEntry("current").setDouble(hoodMotor.getOutputCurrent());


        NetworkTable switchTable = table.getSubTable("limit_switches");
        
        NetworkTable topSwitchTable = switchTable.getSubTable("Hood Max Angle Limit Switch");
        topSwitchTable.getEntry("name").setString("Hood Max Angle Limit Switch");
        topSwitchTable.getEntry("status").setBoolean(topSwitch.get());

        NetworkTable bottomSwitchTable = switchTable.getSubTable("Hood Min Angle Limit Switch");
        bottomSwitchTable.getEntry("name").setString("Hood Min Angle Limit Switch");
        bottomSwitchTable.getEntry("status").setBoolean(bottomSwitch.get());
    }

    public static void reHome() {
        homed = false;
    }
}
