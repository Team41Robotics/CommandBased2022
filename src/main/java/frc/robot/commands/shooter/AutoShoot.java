// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap.ShooterConstants;

public class AutoShoot extends CommandBase {

    private double distance;

    private double angle;
    private double speed;

    public AutoShoot() {
        addRequirements(Robot.hood, Robot.drivetrain, Robot.shooter);
    }

    @Override
    public void execute() {
        distance = Robot.limelight.estimateDistance();
        speed = (distance * ShooterConstants.HOOD_SPEED_SLOPE) + ShooterConstants.HOOD_SPEED_OFFSET;
        angle =
            (distance * distance * ShooterConstants.HOOD_ANGLE_CURVE) +
            (distance * ShooterConstants.HOOD_ANGLE_SLOPE) +
            ShooterConstants.HOOD_ANGLE_OFFSET;
        Robot.limelight.setLedOn(true);
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
        return Robot.shooter.isReady() & Robot.hood.ready & Robot.drivetrain.alignToGoal();
 
    }
}
