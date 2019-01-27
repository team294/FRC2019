/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class Shift extends Command {

  private boolean shift;

  public Shift(boolean high) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.shifter);
    shift = high;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.log.writeLog("Shift", "initialize start", "current time," + System.currentTimeMillis());
    Robot.shifter.setShift(shift);
    Robot.log.writeLog("Shift", "initialize finish", "current time," + System.currentTimeMillis());
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.log.writeLog("Shift", "execute", "current time," + System.currentTimeMillis());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    Robot.log.writeLog("Shift", "isFinished", "current time," + System.currentTimeMillis());
    return true;
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
