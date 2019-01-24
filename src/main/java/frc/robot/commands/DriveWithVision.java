/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class DriveWithVision extends Command {

  private boolean endOnLine = false;

  public DriveWithVision() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveTrain);
  }

  /**
   * Drive towards the vision target
   * @param endOnLine specify whether or not to end on the line target.
   *  </br> true means end on line, false means continue to wall (will not exit with false)
   */
  public DriveWithVision(boolean endOnLine) {
    requires(Robot.driveTrain);
    this.endOnLine = endOnLine;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("---VISION TRACKING INITIATED---");
    Robot.driveTrain.driveToCrosshair();
    //Robot.driveTrain.clearEncoderList();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.driveTrain.driveToCrosshair();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    SmartDashboard.putBoolean("Is Line Present?", Robot.lineFollowing.isLinePresent());
    return endOnLine && Robot.lineFollowing.isLinePresent() && Robot.vision.distanceFromTarget() < 30; // Stops when a line is detected by the line followers
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    // TODO: Add a robot log call here to indicate an end to the vision portion, possibly with a data dump
    Robot.driveTrain.stop();
    System.out.println("---VISION TRACKING ENDED---");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.driveTrain.stop();
  }
}
