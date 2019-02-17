/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class DriveStraight extends Command {
  private double time;
  private double timeStart;
  private double percentPower;
  
/**
 * 
 * @param percentPower power given to motors (negitive to go backwards)
 * @param time time that power is given to motors (in seconds)
 */
  public DriveStraight(double percentPower, double time) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveTrain);
    this.percentPower = percentPower;
    this.time = time;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    timeStart = Timer.getFPGATimestamp();
    Robot.driveTrain.setLeftMotors(percentPower);
    Robot.driveTrain.setRightMotors(percentPower);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.driveTrain.setLeftMotors(percentPower);
    Robot.driveTrain.setRightMotors(percentPower);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(Timer.getFPGATimestamp() - timeStart >= time){
      return true;
    }else {
      return false;
    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveTrain.stopAllMotors();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
