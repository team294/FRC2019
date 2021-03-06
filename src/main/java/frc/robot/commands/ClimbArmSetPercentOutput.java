/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ClimbArmSetPercentOutput extends Command {
  private double percentOutput;

  /**
   * Sets percent power of climb motors.  ***NOTE*** this command does not stop.
   * If the command is interrupted, then the motors stop.
   * @param percentPower between -1.0 and 1.0
   */
  public ClimbArmSetPercentOutput(double percentOutput) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.climb);
    this.percentOutput = percentOutput;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.climb.setClimbMotorPercentOutput(percentOutput);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.climb.setClimbMotorPercentOutput(percentOutput);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.climb.stopClimb();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.climb.stopClimb();
  }
}
