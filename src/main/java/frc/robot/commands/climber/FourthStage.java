// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap.ClimberConstants;

public class FourthStage extends CommandBase {

    public FourthStage() {
        addRequirements(Robot.climber);
    }

    @Override
    public void initialize() {
        Robot.climber.toggleSecondStageRelease();
    }

    @Override
    public void execute() {
        Robot.climber.setSpeed(-ClimberConstants.CLIMBING_SLOW_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ended");
        Robot.climber.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
