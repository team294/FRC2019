/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class WristRaiseUntilStowed extends Command {
  /**
   * Raises wrist slowly until the upper limit switch is triggered.
   * Stops wrist when the upper limit switch is triggered (stowed position).
   */
  public WristRaiseUntilStowed() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.wrist);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.wrist.setWristMotorPercentOutput(0.1);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.wrist.setWristMotorPercentOutput(0.1);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (Robot.wrist.getWristUpperLimit());
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.wrist.stopWrist();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
