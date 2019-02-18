/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.utilities.RobotPreferences;

public class ElevatorMoveToLevel extends Command {

  private double target;
  private boolean targetInches; // true is target in inches, false is target in position
  private RobotPreferences.ElevatorPosition pos;

  /**
   * Moves elevator to target height
   * @param inches target height in inches from floor
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
   * @param pos target height based on the position per RobotPreferences.ElevatorPosition.  
   * If the robot has a ball, then the position is raised as needed for the rocket.
   */
  public ElevatorMoveToLevel(RobotPreferences.ElevatorPosition pos) {
    requires(Robot.elevator);
    this.pos = pos;
    targetInches = false;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if (!targetInches) {
      if (Robot.cargo.getPhotoSwitch()) {
        switch (pos) {
          case bottom:
            target = Robot.robotPrefs.elevatorBottomToFloor;
            break;
          case wristSafe:
            target = Robot.robotPrefs.elevatorWristSafe;
            break;
          case hatchLow:
            target = Robot.robotPrefs.hatchLow + Robot.robotPrefs.rocketBallOffset;
            break;
          case hatchMid:
            target = Robot.robotPrefs.hatchMid + Robot.robotPrefs.rocketBallOffset;
            break;
          case hatchHigh:
            target = Robot.robotPrefs.hatchHigh;
            break;
          case cargoShipCargo:
            target = Robot.robotPrefs.cargoShipCargo;
            break;
          case loadCargo:
            target = Robot.robotPrefs.loadCargo;
            break;
          case groundCargo:
            target = Robot.robotPrefs.groundCargo;
            break;
        }
      } else {
        switch (pos) {
          case bottom:
            target = Robot.robotPrefs.elevatorBottomToFloor;
            break;
          case wristSafe:
            target = Robot.robotPrefs.elevatorWristSafe;
            break;
          case hatchLow:
            target = Robot.robotPrefs.hatchLow;
            break;
          case hatchMid:
            target = Robot.robotPrefs.hatchMid;
            break;
          case hatchHigh:
            target = Robot.robotPrefs.hatchHigh;
            break;
          case cargoShipCargo:
            target = Robot.robotPrefs.cargoShipCargo;
            break;
          case loadCargo:
            target = Robot.robotPrefs.loadCargo;
            break;
          case groundCargo:
            target = Robot.robotPrefs.groundCargo;
            break;
        }
      }
    }
    Robot.elevator.setElevatorPos(target);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.elevator.updateElevatorLog();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
   return !Robot.elevator.getEncOK() || Math.abs(Robot.elevator.getElevatorPos() - target) <= 0.5;
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
