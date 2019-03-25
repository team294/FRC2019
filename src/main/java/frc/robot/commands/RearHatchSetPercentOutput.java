/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class RearHatchSetPercentOutput extends Command {
  private double percent;
  private double seconds;

  /**
   * Run hatch motor for indicated time.
   * @param percent percent output (negative = outtake, positive = intake)
   * @param seconds seconds to run motor for (0 = run forever)
   */
  public RearHatchSetPercentOutput(double percent, double seconds) {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.rearHatch);
    this.percent = percent;
    this.seconds = seconds;
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
    if (timeSinceInitialized() >= seconds) return true;
    else return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    if (seconds > 0) Robot.rearHatch.stopRearHatch();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
