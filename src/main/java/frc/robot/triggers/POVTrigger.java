package frc.robot.triggers;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class POVTrigger extends Trigger {
    private Joystick joystick;
    private int setValue;
    private int POV;

  public POVTrigger(int desiredValue, Joystick POV, int POVNum){
    this.setValue = desiredValue;
    this.joystick = POV;
    this.POV = POVNum;
  }
  @Override
  public boolean get() {
    return joystick.getPOV(POV) == setValue;
    // This returns whether the trigger is active
  }
}