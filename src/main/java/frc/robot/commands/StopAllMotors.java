/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class StopAllMotors extends Command {
  /**
   * Stops elevator, wrist, climb, cargo, and rear hatch motors
   */
  public StopAllMotors() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.elevator);
    requires(Robot.wrist);
    requires(Robot.climb);
    requires(Robot.cargo);
    requires(Robot.rearHatch);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.elevator.stopElevator();
    Robot.wrist.stopWrist();
    Robot.climb.stopClimb();
    Robot.cargo.stopCargoIntake();
    Robot.rearHatch.stopRearHatch();
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
