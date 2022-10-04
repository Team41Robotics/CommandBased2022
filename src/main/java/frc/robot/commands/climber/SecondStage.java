// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap.ClimberConstants;

public class SecondStage extends CommandBase {

    public SecondStage() {
        addRequirements(Robot.climber);
    }

    @Override
    public void initialize() {
        Robot.climber.reset();
        Robot.climber.resetEncoders(0);
    }

    @Override
    public void execute() {
        Robot.climber.setSpeed(ClimberConstants.CLIMBING_SPEED_SECOND_STAGE);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ended");
        Robot.climber.setSpeed(0);
        Robot.climber.lockFirstStage(true);
        Robot.climber.lockSecondStage(false);
        Robot.climber.toggleGearShifter();
    }

    @Override
    public boolean isFinished() {
        return Robot.climber.getEncoder() > 90;
    }
}
