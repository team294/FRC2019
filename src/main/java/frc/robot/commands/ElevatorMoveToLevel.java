/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.utilities.RobotPreferences;

public class ElevatorMoveToLevel extends Command {

  private double target;
  private boolean targetInches; // true is target in inches, false is target in position
  private RobotPreferences.ElevatorPosition pos;

  /**
   * Moves elevator to target height
   * @param inches target height in inches (keep in mind that there's a robotOffset)
   */
  public ElevatorMoveToLevel(double inches) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.elevator);
    target = inches;
    targetInches = true;
  }
  
  /**
   * Moves elevator to target height
   * @param pos target height based on the position called from RobotMap
   */
  public ElevatorMoveToLevel(RobotPreferences.ElevatorPosition pos) {
    requires(Robot.elevator);
    this.pos = pos;
    targetInches = false;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  if(targetInches) {
    Robot.elevator.setElevatorPos(target);
  }
  else {
    /* if(isBall) { //TODO correct if statement when intake subsystem is coded
      switch (pos) {
        case hatchLow:
          Robot.elevator.setElevatorPos(Robot.robotPrefs.hatchLow + RobotMap.ballOffset);
          break;
        case hatchMid:
          Robot.elevator.setElevatorPos(Robot.robotPrefs.hatchMid + RobotMap.ballOffset);
          break;
        case hatchHigh:
          Robot.elevator.setElevatorPos(Robot.robotPrefs.hatchHigh + RobotMap.ballOffset);
          break;
        case cargoShipCargo:
          Robot.elevator.setElevatorPos(Robot.robotPrefs.cargoShipCargo);
          break;
      }
    }
    else { //TODO correct if statement when intake subsystem is coded */
      switch (pos) {
        case hatchLow:
          Robot.elevator.setElevatorPos(Robot.robotPrefs.hatchLow);
          break;
        case hatchMid:
          Robot.elevator.setElevatorPos(Robot.robotPrefs.hatchMid);
          break;
        case hatchHigh:
          Robot.elevator.setElevatorPos(Robot.robotPrefs.hatchHigh);
          break;
        case cargoShipCargo:
          Robot.elevator.setElevatorPos(Robot.robotPrefs.cargoShipCargo);
          break;
      }
    }
  }


  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
   return !Robot.elevator.getEncOK() || Math.abs(Robot.elevator.getElevatorEncInches() - target) <= 0.5;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.elevator.stopElevator();
  }
}
