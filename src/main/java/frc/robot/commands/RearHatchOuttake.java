/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class RearHatchOuttake extends Command {
  private double percent;

  /**
   * Run hatch outtake 5 seconds.
   * @param percent percent output to release hatch, should be negative
   */
  public RearHatchOuttake(double percent) {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.rearHatch);
    this.percent = percent;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.rearHatch.setRearHatchMotorPercentOutput(percent);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.rearHatch.setRearHatchMotorPercentOutput(percent);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (timeSinceInitialized() >= 5) return true;
    else return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.rearHatch.stopRearHatch();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
