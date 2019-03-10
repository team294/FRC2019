/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class TurnWithGyro extends Command {

  double targetAngle;
  boolean isRelativeGyroAngle;
  double originalGyroAngle;

  public TurnWithGyro(double targetAngle, boolean isRelativeAngle) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveTrain);
    this.targetAngle = targetAngle;
    isRelativeGyroAngle = isRelativeAngle;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    originalGyroAngle = Robot.driveTrain.getGyroRotation();
    if(isRelativeGyroAngle == true){
      targetAngle = originalGyroAngle + targetAngle;
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.driveTrain.turnWithGyro(targetAngle);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(isRelativeGyroAngle == false && Math.abs(Robot.driveTrain.getGyroRotation() - targetAngle) <= 3.0){
      return true;
    } else if(isRelativeGyroAngle == true && (originalGyroAngle + targetAngle) - Robot.driveTrain.getGyroRotation() <= 3.0){
      return true;
    } else {
      return false;
    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
