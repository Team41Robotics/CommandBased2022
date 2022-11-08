// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class DisableShooter extends CommandBase {

    public DisableShooter() {
        addRequirements(Robot.hood);
    }

    @Override
    public void execute() {
        Robot.shooter.setSpeed(0);
        Robot.hood.setToPosition(5);
        Robot.shooter.runFeeder(false);

    }

    @Override
    public void end(boolean interrupted) {
        Robot.hood.hoodMotor.set(0);
    }

    @Override
    public boolean isFinished() {
        return Robot.shooter.isReady() && Robot.hood.ready;
    }
}
