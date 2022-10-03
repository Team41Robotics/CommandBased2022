// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/** An example command that uses an example subsystem. */
public class DisableShooter extends CommandBase {

    public DisableShooter() {
        addRequirements(Robot.hood);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        ShooterSubsystem.setSpeed(0);
        HoodSubsystem.setToPosition(5);
    }

    @Override
    public void end(boolean interrupted) {
        HoodSubsystem.hoodMotor.set(0);
    }

    @Override
    public boolean isFinished() {
        return ShooterSubsystem.isReady() && HoodSubsystem.ready;
    }
}
