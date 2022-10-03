// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap.ShooterConstants;

public class ZeroHood extends CommandBase {

    public ZeroHood() {
        addRequirements(Robot.hood);
    }

    @Override
    public void execute() {
        Robot.hood.hoodMotor.set(-ShooterConstants.HOOD_SPEED / 4);
    }

    @Override
    public void end(boolean interrupted) {
        Robot.hood.enc.setPosition(0);
        Robot.hood.hoodMotor.set(0);
    }

    @Override
    public boolean isFinished() {
        return !Robot.hood.getBottomSwitch();
    }
}
