/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ElevatorMoveToBottomThenZeroEncoder extends Command {
  /**
   * Drives the elevator down slowly until it reaches
   * the lower limit switch, then zeros the encoder
   */
  public ElevatorMoveToBottomThenZeroEncoder() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.elevator);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.elevator.setElevatorMotorPercentOutput(-0.2);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    // Stop if climber is at the limit switch, or if we are about to hit the wrist
    return Robot.elevator.getElevatorLowerLimit() || Robot.wrist.getWristAngle() > Robot.robotPrefs.wristKeepOut;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.elevator.stopElevator();
    Robot.elevator.checkAndZeroElevatorEnc();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.elevator.stopElevator();
  }
}
