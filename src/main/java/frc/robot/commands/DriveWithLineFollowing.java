/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class DriveWithLineFollowing extends Command {

  boolean gyro = false;
  double targetQuad = 0;

  public DriveWithLineFollowing() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveTrain);
  }

  /**
   * Drives following line sensors
   * @param gyro Whether or not to use gyro enhancement. True means yes. Default (no parameter) is false.
   */
  public DriveWithLineFollowing(boolean gyro) {
    requires(Robot.driveTrain);
    this.gyro = gyro;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.driveTrain.setDriveMode(false);
    if (gyro) targetQuad = Robot.driveTrain.checkScoringQuadrant(); // Probably should compare this to the quadrant from the vision command too
    Robot.log.writeLogEcho("DriveTrain", "Line Tracking Init", "Gyro," + gyro + ",Quadrant," + targetQuad);
    //Robot.driveTrain.clearEncoderList();
    //Robot.driveTrain.driveOnLine();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.driveTrain.quadrantLineFollowing(targetQuad);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.driveTrain.areEncodersStopped(5.0); // Check if the encoders have changed
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveTrain.stop();
    Robot.log.writeLogEcho("DriveTrain", "Line Tracking Ended", "");
    //Robot.driveTrain.setDriveMode(true); // Might depend on pathfinder or driver preference
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
