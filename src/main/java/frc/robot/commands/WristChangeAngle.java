/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.utilities.RobotPreferences.WristAngle;;

public class WristChangeAngle extends Command {

  private double target;

  /**
   * Moves wrist to target angle
   * @param angle target angle in degrees
   */
  public WristChangeAngle(double angle) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires (Robot.wrist);
    target = angle;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.wrist.setWristAngle(target + Robot.wrist.getCurrentWristTarget());
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.wrist.updateWristLog(false);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return !Robot.wrist.isEncoderCalibrated() || Math.abs(Robot.wrist.getWristAngle() - Robot.wrist.getCurrentWristTarget()) < 5.0; // tolerance of 5 degrees
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    // Robot.wrist.stopWrist();
    // Don't take the wrist out of automated mode if we interrupt a sequence!!!!
  }
}
