// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap.ShooterConstants;

public class PrepareToShoot extends CommandBase {

    public PrepareToShoot() {
        addRequirements(Robot.drivetrain, Robot.shooter, Robot.hood);
    }

    @Override
    public void execute() {
        Robot.limelight.setLedOn(true);
        Robot.drivetrain.setNoRamp(0);

        double distance = Robot.limelight.estimateDistance();
        double speed = (distance * ShooterConstants.HOOD_SPEED_SLOPE) + ShooterConstants.HOOD_SPEED_OFFSET;

        double angle =
            (distance * distance * ShooterConstants.HOOD_ANGLE_CURVE) +
            (distance * ShooterConstants.HOOD_ANGLE_SLOPE) +
            ShooterConstants.HOOD_ANGLE_OFFSET;

        if (Robot.limelight.targetFound()) {
            Robot.shooter.setSpeed(speed / 100);
            Robot.hood.setToPosition(angle);
        }
    }

    @Override
    public void end(boolean interrupted) {
        Robot.hood.hoodMotor.set(0);
    }

    @Override
    public boolean isFinished() {
        return Robot.hood.isReady();
    }
}
