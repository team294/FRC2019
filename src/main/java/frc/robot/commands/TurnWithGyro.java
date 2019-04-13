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

public class TurnWithGyro extends Command {

  double targetAngle;
  double inputAngle;
  boolean isRelativeGyroAngle;
  double originalGyroAngle;

  public TurnWithGyro(double targetAngle, boolean isRelativeAngle) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveTrain);
    inputAngle = targetAngle;
    isRelativeGyroAngle = isRelativeAngle;
    SmartDashboard.putNumber("turn min", 0.1);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.driveTrain.turnwithGyroReset();

    originalGyroAngle = Robot.driveTrain.getGyroRotation();
    if(isRelativeGyroAngle){
      targetAngle = originalGyroAngle + inputAngle;
    } else {
      targetAngle = inputAngle;
    }
    SmartDashboard.putNumber("target angle", targetAngle);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.driveTrain.turnWithGyro(targetAngle);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    System.out.println(Robot.driveTrain.getGyroRotation() + " target= " + targetAngle + " error " + Robot.driveTrain.normalizeAngle(Robot.driveTrain.getGyroRotation() - targetAngle) );
    return Math.abs( Robot.driveTrain.normalizeAngle(Robot.driveTrain.getGyroRotation() - targetAngle) ) <= 1.5;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveTrain.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.driveTrain.stop();
  }
}
