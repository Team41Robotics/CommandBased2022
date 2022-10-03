// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap.ShooterConstants;

public class SetHoodPosition extends CommandBase {

    private double angle;

	/**
	 * @param angle The anlge to set the hood to (Measured in FU)
	 */
    public SetHoodPosition(double angle) {
        this.angle = angle;
        addRequirements(Robot.hood);
    }

    @Override
    public void initialize() {
        Robot.hood.ready = false;
    }

    @Override
    public void execute() {
        double pos = Robot.hood.enc.getPosition();
        if (Robot.hood.topSwitch.get() && (pos - angle) < -1 && pos < ShooterConstants.HOOD_MAX_POS) {
            Robot.hood.hoodMotor.set(ShooterConstants.HOOD_SPEED / 2);
            Robot.hood.ready = false;
        } else if (Robot.hood.bottomSwitch.get() && (pos - angle) > 1 && pos > ShooterConstants.HOOD_MIN_POS) {
            Robot.hood.hoodMotor.set(-ShooterConstants.HOOD_SPEED / 4);
            Robot.hood.ready = false;
        } else {
            Robot.hood.hoodMotor.set(0);
            Robot.hood.ready = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        Robot.hood.hoodMotor.set(0);
    }

    @Override
    public boolean isFinished() {
        return Robot.hood.ready;
    }
}
