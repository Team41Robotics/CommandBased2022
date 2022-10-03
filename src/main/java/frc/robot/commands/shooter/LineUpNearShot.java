// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class LineUpNearShot extends CommandBase {

    private double angle;
    private double speed;

    public LineUpNearShot() {
        addRequirements(Robot.hood, Robot.drivetrain);
    }

    @Override
    public void execute() {
        speed = 32.5;
        angle = 15;

        Robot.shooter.setSpeed(speed / 100);
        Robot.hood.setToPosition(angle);
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
