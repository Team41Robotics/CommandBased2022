// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap.ShooterConstants;

public class ShootBall extends CommandBase {

    public ShootBall() {
        addRequirements(Robot.intake, Robot.shooter);
    }

    @Override
    public void execute() {
        Robot.shooter.runElevator(ShooterConstants.ELEVATOR_FULL_SPEED);
        Robot.shooter.runFeeder(true);
        Robot.intake.enable();
    }

    @Override
    public void end(boolean interrupted) {
        Robot.shooter.runElevator(0);
        Robot.shooter.runFeeder(false);
        Robot.intake.disable();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
