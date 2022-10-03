// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap.AutonConstants;
import frc.robot.RobotMap.DrivetrainConstants;
import frc.robot.utils.PhotonCamera;

public class GoToBallCareful extends CommandBase {

    private boolean thirdBallClose = false;

    public GoToBallCareful() {
        addRequirements(Robot.drivetrain);
    }

    @Override
    public void execute() {
        if (PhotonCamera.getArea() > 5) {
            thirdBallClose = true;
        }
        double angle = PhotonCamera.getYaw();
        double slowSpeed = 0.5;
        Robot.drivetrain.runInverseKinematics(
            (DrivetrainConstants.BALL_FOLLOWING_kP * -angle),
            !thirdBallClose ? AutonConstants.AUTON_SPEED_M_PER_S : slowSpeed
        );
    }

    @Override
    public void end(boolean interrupted) {
		// Finished when a ball breaks the sensor
        Robot.drivetrain.setNoRamp(0);
    }

    @Override
    public boolean isFinished() {
        return Robot.drivetrain.getDanger() || !Robot.beamBreak.get();
    }
}
