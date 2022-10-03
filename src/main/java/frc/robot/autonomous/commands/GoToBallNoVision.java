// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap.AutonConstants;
import frc.robot.RobotMap.ShooterConstants;

public class GoToBallNoVision extends CommandBase {

    public GoToBallNoVision() {
        addRequirements(Robot.drivetrain, Robot.shooter);
    }

    @Override
    public void initialize() {
        Robot.shooter.setSpeed(ShooterConstants.SHOOTER_DEFAULT_SPEED);
        Robot.limelight.setLedOn(true);
    }

    @Override
    public void execute() {
        // Beam break here

        Robot.drivetrain.runInverseKinematics(0, AutonConstants.AUTON_SPEED_M_PER_S * (2.0 / 3));
    }

    @Override
    public void end(boolean interrupted) {
        do {
            Robot.drivetrain.setPosition(0);
        } while (Robot.drivetrain.getPosition() > 1);
    }

    @Override
    public boolean isFinished() {
        // Finished when a ball breaks the sensor
        return !Robot.beamBreak.get();
    }
}
