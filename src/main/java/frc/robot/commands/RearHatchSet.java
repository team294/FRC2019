/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class RearHatchSet extends Command {
  private boolean extend;

  /**
   * Extend or retract the rear hatch mechanism. Runs the hatch motor
   * outtake for three seconds if going to stowed position (just in case we are holding a hatch).
   * @param grab true = extended position, false = retracted position
   */
  public RearHatchSet(boolean extend) {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.rearHatch);
    this.extend = extend;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if (extend) {
      Robot.rearHatch.setRearHatchPiston(true);
    } else {
      Robot.rearHatch.setRearHatchMotorPercentOutput(-0.6);
      Robot.rearHatch.setRearHatchPiston(false);
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (extend || timeSinceInitialized() >= 1.5);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    if (!extend) Robot.rearHatch.stopRearHatch();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
