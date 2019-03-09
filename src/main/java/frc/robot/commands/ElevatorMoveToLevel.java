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

  private double timeAtLooseTolerance = 0; // the amount of time we are within the loose tolerance
  private boolean startTime = false; // have we started to count the time for loose tolerance? 

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
    startTime = false;        // We haven't hit the loose tolerance yet

    if(!targetInches) {
      if(Robot.cargo.hasBall()) { 
        switch (pos) {
          case bottom:
            target = Robot.robotPrefs.elevatorBottomToFloor;
            break;
          case wristStow:
            target = Robot.robotPrefs.elevatorWristStow;
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
          case wristStow:
            target = Robot.robotPrefs.elevatorWristStow;
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
    Robot.elevator.startPID(target);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // Robot.elevator.updateElevatorLog(false);

    // // Start timer for loose tolerance
    // if(!startTime && Math.abs(Robot.elevator.getElevatorPos() - target) <= 2.5) {
    //   timeAtLooseTolerance = timeSinceInitialized();
    //   startTime = true;
    // }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
  return true;
  //  return !Robot.elevator.getEncOK() ||         // End immediately if encoder can't read
  //    Math.abs(Robot.elevator.getElevatorPos() - target) <= 0.5 ||
  //    (startTime && timeSinceInitialized() - timeAtLooseTolerance > 0.5);
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
