// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap.DrivetrainConstants;

/** An example command that uses an example subsystem. */
public class DefaultDrive extends CommandBase {

    public DefaultDrive() {
        addRequirements(Robot.drivetrain);
    }

    @Override
    public void execute() {
        // TODO - limit drive speed during climb

        double deadband = Robot.climber.climbing
            ? DrivetrainConstants.JOYSTICK_CLIMBING_MODE_DEADZONE
            : DrivetrainConstants.JOYSTICK_DEADZONE;

        double leftSpeed = MathUtil.applyDeadband(-Robot.leftJoystick.getY(), deadband);
        double rightSpeed = MathUtil.applyDeadband(-Robot.rightJoystick.getY(), deadband);

        double speedOffset = (-Robot.operatorConsole.getRawAxis(0) + 1) / 2.0;

        Robot.drivetrain.drive(leftSpeed * speedOffset, rightSpeed * speedOffset);
    }
}
