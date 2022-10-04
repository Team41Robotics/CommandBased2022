package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.ShooterConstants;

public class HoodSubsystem extends SubsystemBase {

    public boolean ready, homed;
    public double angle;
    public CANSparkMax hoodMotor;
    public DigitalInput topSwitch, bottomSwitch;
    public RelativeEncoder enc;

    /**
     * Initialize all motors, encoders, and switches needed to operate the hood
     */
    public HoodSubsystem() {
        // Bottom switch needs to be inverted
        hoodMotor = new CANSparkMax(ShooterConstants.HOOD_SPARK, MotorType.kBrushless);
        topSwitch = new DigitalInput(ShooterConstants.HOOD_TOP_LIMIT_SWITCH);
        bottomSwitch = new DigitalInput(ShooterConstants.HOOD_BOTTOM_LIMIT_SWITCH);
        hoodMotor.setIdleMode(IdleMode.kBrake);
        enc = hoodMotor.getEncoder();
        enc.setPosition(0);
        angle = 0;
        ready = false;
        homed = false;
    }

    /**
     * Sets the hood to a specified position
     * @param angle the desired angle of the hood (in rotations of the motor)
     */
    public void setToPosition(double angle) {
        double pos = enc.getPosition();
        if (topSwitch.get() && (pos - angle) < -1 && pos < ShooterConstants.HOOD_MAX_POS) {
            hoodMotor.set(ShooterConstants.HOOD_SPEED / 2);
            ready = false;
        } else if (bottomSwitch.get() && (pos - angle) > 1 && pos > ShooterConstants.HOOD_MIN_POS) {
            hoodMotor.set(-ShooterConstants.HOOD_SPEED / 4);
            ready = false;
        } else {
            hoodMotor.set(0);
            ready = true;
        }
    }

    /**
     * Homes the hood asynchronously (needs to be called more than once to finish)
     */
    public void home() {
        if (bottomSwitch.get()) {
            hoodMotor.set(-ShooterConstants.HOOD_SPEED / 4);
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
    public boolean isHomed() {
        return homed;
    }

    /**
     * Get the ready status of the hood
     * @return Whether the hood is in the desired position
     */
    public boolean isReady() {
        return ready;
    }

    /**
     * Get the value of the hood limit switch at the bottom of its motion
     * @return the value of the switch
     */
    public boolean getBottomSwitch() {
        return bottomSwitch.get();
    }

    /**
     * Get the value of the hood limit switch at the top of its motion
     * @return the value of the switch
     */
    public boolean getTopSwitch() {
        return topSwitch.get();
    }

    /**
     * Upload all telemetry data for the hood
     * @param table the base telemetry NetworkTable
     */
    public void telemetry(NetworkTable table) {
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

    public void reHome() {
        homed = false;
    }
}
