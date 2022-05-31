package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap.INTAKE;
import frc.robot.RobotMap.INTAKE_MODE;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
* Controls the intake on the robot 
*/
public class IntakeSubsystem extends SubsystemBase{
    private static boolean intakeOn;
    //private static boolean intakeUp;
    private static CANSparkMax intakeMotor, conveyor;
    private static DoubleSolenoid intakeSolLeft;
    private static DoubleSolenoid intakeSolRight;
    //private static Joystick leftJoy, rightJoy, secondDS;
    public static INTAKE_MODE direction;
    /**
    * Initialize all parts of the intake
    */
    public static void initIntake(){
        intakeOn = false;
/*      intakeUp = false;
        leftJoy = Robot.leftJoy;
        rightJoy = Robot.rightJoy;
        secondDS = Robot.secondDS; */
        intakeMotor = new CANSparkMax(INTAKE.INTAKE_MOTOR, MotorType.kBrushless);
        conveyor = new CANSparkMax(INTAKE.CONVEYOR_MOTOR, MotorType.kBrushless);
        conveyor.setInverted(true);
        intakeMotor.setIdleMode(IdleMode.kCoast);
        intakeSolLeft = new DoubleSolenoid(INTAKE.PCM_PORT, PneumaticsModuleType.REVPH, INTAKE.LEFT_SOL_FWD, INTAKE.LEFT_SOL_RV);
        intakeSolRight = new DoubleSolenoid(INTAKE.PCM_PORT, PneumaticsModuleType.REVPH, INTAKE.RIGHT_SOL_FWD, INTAKE.RIGHT_SOL_RV);
        SmartDashboard.putBoolean("Intake On", false);
        intakeSolLeft.set(DoubleSolenoid.Value.kForward);
        intakeSolRight.set(DoubleSolenoid.Value.kForward);
        direction = INTAKE_MODE.FORWARD;
    }
    
    /**
    * At beginning of auton, move the intake down and start the motor
    */
    public static void autonInit(){
        intakeSolLeft.set(DoubleSolenoid.Value.kForward);
        intakeSolRight.set(DoubleSolenoid.Value.kForward);
        //intakeUp = true;
        intakeMotor.set(INTAKE.INTAKE_FULL_SPEED);
        conveyor.set(INTAKE.CONVEYOR_FULL_SPEED);
    }
    
    /**
    * In teleop, use joystick triggers to raise/lower the intake and toggle the motor
    */

    
    /**
    * Place to put intake test code
    */
    public static void test() {
        /*
        if (leftJoy.getRawButtonPressed(Controls.LeftJoy.INTAKE_PISTON_TOGGLE)) {
            intakeUp = !intakeUp;
            intakeSolLeft.set(intakeUp ? DoubleSolenoid.Value.kReverse : DoubleSolenoid.Value.kForward);
            intakeSolRight.set(intakeUp ? DoubleSolenoid.Value.kReverse : DoubleSolenoid.Value.kForward);
        }
        if (rightJoy.getRawButtonPressed(Controls.RightJoy.INTAKE_TOGGLE)) {
            intakeOn = !intakeOn;
            run(intakeOn ? INTAKE_MODE.FORWARD : INTAKE_MODE.OFF);
        }
        */
    }
    
    /**
    * Run the intake and conveyor
    * @param dir the direction to run the intake (enum)
    */
    public static void run() {
        System.out.println(direction);
        if(intakeOn){
        switch (direction) {
            case FORWARD:
            intakeMotor.set(INTAKE.INTAKE_FULL_SPEED);
            conveyor.set(INTAKE.CONVEYOR_FULL_SPEED);
            break;
            case REVERSE:
            intakeMotor.set(-INTAKE.INTAKE_FULL_SPEED);
            conveyor.set(-INTAKE.CONVEYOR_FULL_SPEED);

        }
    }
        else{
            intakeMotor.set(0);
            conveyor.set(0);
    }
    }
   
    /**
    * Set whether or not the conveyor should be on
    * @param on true to turn the conveyor on, false to turn it off
    */
    public static void runConveyor(boolean on) {
        conveyor.set(on ? INTAKE.CONVEYOR_FULL_SPEED : 0);
    }
    
    /**
    * Stop all motors controlled by the intake
    */
    public static void stop() {
        conveyor.set(0);
        intakeMotor.set(0);
    }
    
    /**
    * Make the intake solenoids put the intake in the up position
    */
    public static void putUp() {
        intakeSolLeft.set(Value.kReverse);
        intakeSolRight.set(Value.kReverse);
    }
    
    public static void putDown() {
        intakeSolLeft.set(Value.kForward);
        intakeSolRight.set(Value.kForward);
    }

    public static void toggle(){
        if(intakeSolLeft.get() == Value.kForward){
            putUp();
        }
        else{
            putDown();
        }
    }

    public static void toggleMotors(){
        intakeOn = !intakeOn;
        System.out.println(intakeOn);
    }


    /**
    * Reset the intake to a known position and state
    */
    public static void reset() {
        putUp();
        stop();
        intakeOn = false;
        //intakeUp = false;
    }
    
    /**
    * Record all telemetry data for the intake
    * @param table the base telemetry NetworkTable
    */
    public static void telemetry(NetworkTable table) {
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
        conveyorMotorTable.getEntry("raw_input_speed").setDouble(conveyor.get());
        conveyorMotorTable.getEntry("output_speed").setDouble(conveyor.getEncoder().getVelocity());
        conveyorMotorTable.getEntry("position").setDouble(conveyor.getEncoder().getPosition());
        conveyorMotorTable.getEntry("current").setDouble(conveyor.getOutputCurrent());
        
        
        NetworkTable solenoidTable = table.getSubTable("solenoids");
        
        NetworkTable intakeSolLeftTable = solenoidTable.getSubTable("Left Intake Solenoid");
        intakeSolLeftTable.getEntry("name").setString("Left Intake Solenoid");
        intakeSolLeftTable.getEntry("status").setDouble(intakeSolLeft.get() == Value.kForward ? 1 : (intakeSolLeft.get() == Value.kReverse ? -1 : 0));
        
        NetworkTable intakeSolRightTable = solenoidTable.getSubTable("Right Intake Solenoid");
        intakeSolRightTable.getEntry("name").setString("Right Intake Solenoid");
        intakeSolRightTable.getEntry("status").setDouble(intakeSolRight.get() == Value.kForward ? 1 : (intakeSolRight.get() == Value.kReverse ? -1 : 0));
    }
}   

