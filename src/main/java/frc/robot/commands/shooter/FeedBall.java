// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class FeedBall extends CommandBase {

    public FeedBall() {
        addRequirements(Robot.shooter);
    }

    @Override
    public void execute() {
        Robot.shooter.runFeeder(true);
    }

    @Override
    public void end(boolean interrupted) {
        Robot.shooter.runFeeder(false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
