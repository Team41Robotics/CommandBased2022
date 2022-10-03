// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class RunFeeder extends CommandBase {

    private boolean active;

    public RunFeeder(boolean on) {
        active = on;
        addRequirements(Robot.shooter);
    }

    @Override
    public void initialize() {
        Robot.shooter.runFeeder(active);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
