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

/**
 * drives straight based on percent output and time
 */
public class DriveStraightOutputTime extends Command {
  private double time;
  private double timeStart;
  private double percentOutput;
  private boolean sucess;
  
/**
 * 
 * @param percentOutput power given to motors (negative to go backwards)
 * @param time time that power is given to motors (in seconds)
 */
  public DriveStraightOutputTime(double percentOutput, double time) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveTrain);
    this.percentOutput = percentOutput;
    this.time = time;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    timeStart = Timer.getFPGATimestamp();
    Robot.driveTrain.setLeftMotors(percentOutput);
    Robot.driveTrain.setRightMotors(percentOutput);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.driveTrain.setLeftMotors(percentOutput);
    Robot.driveTrain.setRightMotors(percentOutput);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(Timer.getFPGATimestamp() - timeStart >= time || sucess){
      return true;
    }else {
      return false;
    }
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
  }
}
