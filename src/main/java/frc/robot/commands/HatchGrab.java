/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class HatchGrab extends Command {
  private boolean grab;

  /**
   * Grab or releae the hatch claw
   * @param grab true = grab position, false = release position
   */
  public HatchGrab(boolean grab) {
    requires(Robot.hatch);
    this.grab = grab;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if (grab) {
      Robot.hatch.grabHatch();
    } else {
      Robot.hatch.releaseHatch();
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
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
