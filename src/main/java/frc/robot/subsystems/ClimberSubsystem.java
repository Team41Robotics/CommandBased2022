package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.ClimberConstants;

/**
 * Pistons:
 * <ul>
 * <li>secondStageGearLock: fwd - locked; rev - unlocked</li>
 * <li>firstStageGearLock: fwd - locked; rev - unlocked</li>
 * <li>secondStageRelease: fwd - locked; rev - unlocked</li>
 * <li>gearShifter: fwd - first stage; rev - second stage</li>
 * </ul>
 */
public class ClimberSubsystem extends SubsystemBase {

    public boolean climbing;
    // private boolean firstStageUp, secondStageUp;
    private int climbingState;
    private double motorSpeed;
    // private statc long startTime;
    private CANSparkMax climbingMotor1;
    private CANSparkMax climbingMotor2;
    private DigitalInput firstStageLeftSwitch, firstStageRightSwitch, secondStageSwitch, secondStageSecondSwitch;
    private DoubleSolenoid secondStageGearLock, firstStageGearLock, secondStageRelease, gearShifter; // secondStageRelease

    private RelativeEncoder climbingEncoder1;
    private RelativeEncoder climbingEncoder2;

    public ClimberSubsystem() {
        climbingMotor1 = new CANSparkMax(ClimberConstants.CLIMBING_SPARK_F, MotorType.kBrushless);
        climbingMotor2 = new CANSparkMax(ClimberConstants.CLIMBING_SPARK_B, MotorType.kBrushless);
        climbingMotor1.setIdleMode(IdleMode.kBrake);
        climbingMotor2.setIdleMode(IdleMode.kBrake);

        secondStageGearLock =
            new DoubleSolenoid(
                ClimberConstants.PCM_PORT,
                PneumaticsModuleType.REVPH,
                ClimberConstants.INNER_GEAR_LOCK_OFF,
                ClimberConstants.INNER_GEAR_LOCK_ON
            );
        firstStageGearLock =
            new DoubleSolenoid(
                ClimberConstants.PCM_PORT,
                PneumaticsModuleType.REVPH,
                ClimberConstants.OUTER_GEAR_LOCK_OFF,
                ClimberConstants.OUTER_GEAR_LOCK_ON
            );
        secondStageRelease =
            new DoubleSolenoid(
                ClimberConstants.PCM_PORT,
                PneumaticsModuleType.REVPH,
                ClimberConstants.MIDDLE_ARM_LOCK,
                ClimberConstants.MIDDLE_ARM_RELEASE
            );
        gearShifter =
            new DoubleSolenoid(
                ClimberConstants.PCM_PORT,
                PneumaticsModuleType.REVPH,
                ClimberConstants.MOVE_TO_OUTER_ARMS,
                ClimberConstants.MOVE_TO_INNER_ARM
            );
        climbingEncoder1 = climbingMotor1.getEncoder();
        climbingEncoder2 = climbingMotor2.getEncoder();

        // firstStageUp = false;
        // secondStageUp = false;

        secondStageGearLock.set(DoubleSolenoid.Value.kForward);
        secondStageRelease.set(DoubleSolenoid.Value.kForward);
        firstStageGearLock.set(DoubleSolenoid.Value.kReverse);
        gearShifter.set(Value.kForward);

        firstStageLeftSwitch = new DigitalInput(ClimberConstants.FIRST_STAGE_LIMIT_SWITCH_L);
        firstStageRightSwitch = new DigitalInput(ClimberConstants.FIRST_STAGE_LIMIT_SWITCH_R);
        secondStageSecondSwitch = new DigitalInput(ClimberConstants.FIRST_STAGE_LIMIT_SWTICH_M);
        secondStageSwitch = new DigitalInput(ClimberConstants.SECOND_STAGE_LIMIT_SWITCH);

        climbing = false;
    }

    /**
     * Reset the climbing process
     */
    public void reset() {
        secondStageGearLock.set(DoubleSolenoid.Value.kForward);
        secondStageRelease.set(DoubleSolenoid.Value.kForward);
        firstStageGearLock.set(DoubleSolenoid.Value.kReverse);
        gearShifter.set(Value.kForward);
        // firstStageUp = false;
        // secondStageUp = false;
        climbing = false;
    }

    /**
     * Set the speed of the climber motors
     *
     * @param speed The speed of the climber motors
     */
    public void setSpeed(double speed) {
        motorSpeed = speed;
        climbingMotor1.set(motorSpeed);
        climbingMotor2.set(motorSpeed);
    }

    /**
     * Toggle the Second Stage Gear Lock
     */
    public void toggleSecondStageLock() {
        if (secondStageGearLock.get() == DoubleSolenoid.Value.kForward) {
            secondStageGearLock.set(DoubleSolenoid.Value.kReverse);
        } else {
            secondStageGearLock.set(DoubleSolenoid.Value.kForward);
        }
    }

    /**
     * Toggle the First Stage Gear Lock
     */
    public void toggleFirstStageLock() {
        if (firstStageGearLock.get() == DoubleSolenoid.Value.kReverse) {
            firstStageGearLock.set(DoubleSolenoid.Value.kForward);
        } else {
            firstStageGearLock.set(DoubleSolenoid.Value.kReverse);
        }
    }

    public void toggleSecondStageRelease() {
        if (secondStageRelease.get() == DoubleSolenoid.Value.kForward) {
            secondStageRelease.set(DoubleSolenoid.Value.kReverse);
        } else {
            secondStageRelease.set(DoubleSolenoid.Value.kForward);
        }
    }

    public void toggleGearShifter() {
        System.out.println("here");
        if (gearShifter.get() == Value.kForward) {
            gearShifter.set(Value.kReverse);
        } else {
            gearShifter.set(Value.kForward);
        }
    }

    public void resetEncoders(int position) {
        climbingEncoder1.setPosition(position);
        climbingEncoder2.setPosition(position);
    }

    public double getEncoder() {
        return climbingEncoder1.getPosition();
    }

    public void lockFirstStage(boolean forward) {
        if (forward) {
            firstStageGearLock.set(DoubleSolenoid.Value.kForward);
        } else {
            firstStageGearLock.set(DoubleSolenoid.Value.kReverse);
        }
    }

    public void lockSecondStage(boolean forward) {
        if (forward) {
            secondStageGearLock.set(DoubleSolenoid.Value.kForward);
        } else {
            secondStageGearLock.set(DoubleSolenoid.Value.kReverse);
        }
    }

    public void setGearShifter(boolean forward) {
        if (forward) {
            gearShifter.set(DoubleSolenoid.Value.kForward);
        } else {
            gearShifter.set(DoubleSolenoid.Value.kReverse);
        }
    }

    /**
     * Get the value of the leftmost limit switch
     *
     * @return the value of the switch
     */
    public boolean getLSwitch() {
        return firstStageLeftSwitch.get();
    }

    /**
     * Get the value of the rightmost switch
     *
     * @return the value of the switch
     */
    public boolean getRSwitch() {
        return firstStageRightSwitch.get();
    }

    /**
     * Get the value of the second switch for the middle climber
     *
     * @return the value of the switch
     */
    public boolean getSecondMSwitch() {
        return secondStageSecondSwitch.get();
    }

    /**
     * Get the value of the first switch for the middle climber
     *
     * @return the value of the switch
     */
    public boolean getMSwitch() {
        return secondStageSwitch.get();
    }

    /**
     * Add all telemetry data for the climber
     *
     * @param table the base telemetry networktable
     */
    public void telemetry(NetworkTable table) {
        NetworkTable motorTable = table.getSubTable("motors");

        NetworkTable climberMotor1Table = motorTable.getSubTable("Climber Motor 1");
        climberMotor1Table.getEntry("name").setString("Climber Motor 1");
        climberMotor1Table.getEntry("loop_error").setDouble(-1);
        climberMotor1Table.getEntry("p").setDouble(-1);
        climberMotor1Table.getEntry("i").setDouble(-1);
        climberMotor1Table.getEntry("d").setDouble(-1);
        climberMotor1Table.getEntry("requested_input_speed").setDouble(-1);
        climberMotor1Table.getEntry("actual_input_speed").setDouble(-1);
        climberMotor1Table.getEntry("raw_input_speed").setDouble(climbingMotor1.get());
        climberMotor1Table.getEntry("output_speed").setDouble(climbingMotor1.getEncoder().getVelocity());
        climberMotor1Table.getEntry("position").setDouble(climbingMotor1.getEncoder().getPosition());
        climberMotor1Table.getEntry("current").setDouble(climbingMotor1.getOutputCurrent());

        NetworkTable climberMotor2Table = motorTable.getSubTable("Climber Motor 2");
        climberMotor2Table.getEntry("name").setString("Climber Motor 2");
        climberMotor2Table.getEntry("loop_error").setDouble(-1);
        climberMotor2Table.getEntry("p").setDouble(-1);
        climberMotor2Table.getEntry("i").setDouble(-1);
        climberMotor2Table.getEntry("d").setDouble(-1);
        climberMotor2Table.getEntry("requested_input_speed").setDouble(-1);
        climberMotor2Table.getEntry("actual_input_speed").setDouble(-1);
        climberMotor2Table.getEntry("raw_input_speed").setDouble(climbingMotor2.get());
        climberMotor2Table.getEntry("output_speed").setDouble(climbingMotor2.getEncoder().getVelocity());
        climberMotor2Table.getEntry("position").setDouble(climbingMotor2.getEncoder().getPosition());
        climberMotor2Table.getEntry("current").setDouble(climbingMotor2.getOutputCurrent());

        NetworkTable switchTable = table.getSubTable("limit_switches");

        NetworkTable firstStageLeftSwitchTable = switchTable.getSubTable("First Climber Stage Left Limit Switch");
        firstStageLeftSwitchTable.getEntry("name").setString("First Climber Stage Left Limit Switch");
        firstStageLeftSwitchTable.getEntry("status").setBoolean(firstStageLeftSwitch.get());

        NetworkTable firstStageRightSwitchTable = switchTable.getSubTable("First Climber Stage Right Limit Switch");
        firstStageRightSwitchTable.getEntry("name").setString("First Climber Stage Right Limit Switch");
        firstStageRightSwitchTable.getEntry("status").setBoolean(firstStageRightSwitch.get());

        NetworkTable secondStageSwitchTable = switchTable.getSubTable("Second Climber Stage Limit Switch");
        secondStageSwitchTable.getEntry("name").setString("Second Climber Stage Limit Switch");
        secondStageSwitchTable.getEntry("status").setBoolean(secondStageSwitch.get());

        NetworkTable secondStageSecondSwitchTable = switchTable.getSubTable("Second Climber Stage Second Limit Switch");
        secondStageSecondSwitchTable.getEntry("name").setString("Second Climber Stage Second Limit Switch");
        secondStageSecondSwitchTable.getEntry("status").setBoolean(secondStageSecondSwitch.get());

        NetworkTable solenoidTable = table.getSubTable("solenoids");

        NetworkTable secondStageGearLockTable = solenoidTable.getSubTable("Second Stage Friction Brake");
        secondStageGearLockTable.getEntry("name").setString("Second Stage Friction Brake");
        secondStageGearLockTable
            .getEntry("status")
            .setDouble(
                secondStageGearLock.get() == Value.kForward ? 1 : (secondStageGearLock.get() == Value.kReverse ? -1 : 0)
            );

        NetworkTable firstStageGearLockTable = solenoidTable.getSubTable("Gear Jammer");
        firstStageGearLockTable.getEntry("name").setString("Gear Jammer");
        firstStageGearLockTable
            .getEntry("status")
            .setDouble(
                firstStageGearLock.get() == Value.kForward ? 1 : (firstStageGearLock.get() == Value.kReverse ? -1 : 0)
            );

        NetworkTable secondStageReleaseTable = solenoidTable.getSubTable("Second Stage Release");
        secondStageReleaseTable.getEntry("name").setString("Second Stage Release");
        secondStageReleaseTable
            .getEntry("status")
            .setDouble(
                secondStageRelease.get() == Value.kForward ? 1 : (secondStageRelease.get() == Value.kReverse ? -1 : 0)
            );

        NetworkTable gearShifterTable = solenoidTable.getSubTable("Gear Shifter");
        gearShifterTable.getEntry("name").setString("Gear Shifter");
        gearShifterTable
            .getEntry("status")
            .setDouble(gearShifter.get() == Value.kForward ? 1 : (gearShifter.get() == Value.kReverse ? -1 : 0));

        NetworkTableEntry climbingStateEntry = table.getEntry("state_climber");
        switch (climbingState) {
            case 0:
                climbingStateEntry.setDouble(-1);
                break;
            case 45:
                climbingStateEntry.setDouble(0);
                break;
            case 90:
                climbingStateEntry.setDouble(1);
                break;
            case 135:
                climbingStateEntry.setDouble(2);
                break;
            case 180:
                climbingStateEntry.setDouble(3);
                break;
            case 225:
                climbingStateEntry.setDouble(4);
        }
    }
}
