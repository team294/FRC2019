/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 * An example command. You can replace me with your own command.
 */
public class DriveWithJoysticks extends Command {
  public DriveWithJoysticks() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.log.writeLogEcho("DriveTrain", "Driver Control Init", "");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double leftValue = Robot.oi.leftJoystick.getY();
    double rightValue = Robot.oi.rightJoystick.getY();

    if (Robot.oi.getDriveDirection() == true)  {
      Robot.driveTrain.tankDrive(-leftValue, -rightValue);
    } else {
      Robot.driveTrain.tankDrive(leftValue, rightValue);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveTrain.stop();
    Robot.log.writeLogEcho("DriveTrain", "Driver Control Ended", "");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
