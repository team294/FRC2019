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
  private boolean gyro = false;
  private double targetQuad = 0; // The quadrant of the target we want to drive to

  /**
   * Vision assisted driving without gyro, keep going and never end on the line
   */
  public DriveWithVision() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveTrain);
  }

  /**
   * Drive towards the vision target
   * @param endOnLine specify whether or not to end on the line target.
   * @param gyro specify whether or not to use gyro curve correction
   *  </br> true means end on line, false means continue to wall (will not exit with false)
   */
  public DriveWithVision(boolean endOnLine, boolean gyro) {
    requires(Robot.driveTrain);
    this.endOnLine = endOnLine;
    this.gyro = gyro;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    SmartDashboard.putBoolean("Ready to Score", false);
    Robot.driveTrain.clearEncoderList(); // May not be necessary to clear
    //Robot.driveTrain.driveToCrosshair();
    if (gyro) targetQuad = Robot.driveTrain.checkScoringQuadrant();
    System.out.println("Target Quadrant:" + targetQuad);
    Robot.log.writeLogEcho("DriveTrain", "Vision Tracking Init", "Gyro," + gyro + ",Quadrant,"+targetQuad);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.driveTrain.driveToCrosshair(targetQuad);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    Robot.driveTrain.areEncodersStopped(5.0);
    return endOnLine && Robot.lineFollowing.isLinePresent() && Robot.vision.distanceFromTarget() < 40; // Stops when a line is detected by the line followers within a reasonable expected distance
    // TODO:: with an accurate distance measurement, we can stop automatically when close enough
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveTrain.stop();
    Robot.log.writeLogEcho("DriveTrain", "Vision Tracking Ended", "");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
