// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;
import java.lang.Math;

public class MoveForward extends CommandBase {

    private double startPos;

    private double endPos;
    private double movementSpeed;

    public MoveForward(double distance, double speed) {
        endPos = distance;
        movementSpeed = speed;
    }

    @Override
    public void initialize() {
        startPos = Robot.drivetrain.getPosition();
    }

    @Override
    public void execute() {
        Robot.drivetrain.runInverseKinematics(0.075, movementSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        Robot.drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        return Math.abs((startPos - Robot.drivetrain.getPosition()) - endPos) < 1;
    }
}
