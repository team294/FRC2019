/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * changes vision pipeline (0 = vision, 2 = driver feed)
 */
public class VisionChangePipeline extends Command {
  double pipeline;

  /**
  * changes vision pipeline (0 = vision, 2 = driver feed)
  */
  public VisionChangePipeline(double pipeline) {
    // Use requires() here to declare subsystem dependencies
    this.pipeline = pipeline;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.vision.setPipe(pipeline);
    // Robot.vision.setLedMode(3 - (int)pipeline);
    if (pipeline == 0 || pipeline == 1) Robot.vision.setLedMode(3);
    else if (pipeline == 2) Robot.vision.setLedMode(1);
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
