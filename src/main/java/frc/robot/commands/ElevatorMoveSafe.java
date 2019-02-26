/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import frc.robot.Robot;
import frc.robot.utilities.RobotPreferences;
import frc.robot.utilities.RobotPreferences.ElevatorPosition;
import frc.robot.utilities.RobotPreferences.WristAngle;

public class ElevatorMoveSafe extends CommandGroup {


  /**
   * Moves elevator to target height.  If needed, moves climber and wrist
   * to safe positions before moving the elevator.  Always deploys wrist if stowed.
   * @param inches target height in inches from floor
   */
  public ElevatorMoveSafe(double inches) {
    deployWristIfStowed();
    if (inches < Robot.robotPrefs.groundCargo) {
      raiseWristIfBelowHorizontal();
    } else {
      raiseWristIfBelowDown();
    }
    addSequential(new ElevatorMoveToLevel(inches));
  }

  /**
   * Moves elevator to target position.  If needed, moves the climber and wrist
   * to safe positions before moving the elevtor.  Always deploys wrist if stowed.
   * @param pos target height based on the position per RobotPreferences.ElevatorPosition.  
   * If the robot has a ball, then the position is raised as needed for the rocket.
   */
  public ElevatorMoveSafe(RobotPreferences.ElevatorPosition pos) {
    deployWristIfStowed();
    if (pos == ElevatorPosition.bottom || pos == ElevatorPosition.hatchLow || pos == ElevatorPosition.wristStow) {
      raiseWristIfBelowHorizontal();
    } else if (pos == ElevatorPosition.groundCargo) {
      raiseWristIfBelowDown();
    }
    addSequential(new ElevatorMoveToLevel(pos));
  }

  private void deployWristIfStowed() {
    // Move climber if we need to deploy the wrist and the climber is in the way
    addSequential(new ConditionalCommand(new ClimbArmSetAngle(Robot.robotPrefs.climbWristMovingSafe - 5.0)){
      @Override
      protected boolean condition() {
        return Robot.wrist.getWristAngle() > Robot.robotPrefs.wristKeepOut &&
          Robot.climb.getClimbAngle() > Robot.robotPrefs.climbWristMovingSafe;
      }
    });

    // Deploy wrist if stowed
    addSequential(new ConditionalCommand(new WristMoveToAngle(WristAngle.straight)){
      @Override
      protected boolean condition() {
        return Robot.wrist.getWristAngle() > Robot.robotPrefs.wristKeepOut;
      }
    });
  }

  private void raiseWristIfBelowHorizontal() {
    // Raise wrist if below horizontal
    addSequential(new ConditionalCommand(new WristMoveToAngle(WristAngle.straight)){
      @Override
      protected boolean condition() {
        return Robot.wrist.getWristAngle() < Robot.robotPrefs.wristStraight - 5.0;
      }
    });
  }

  private void raiseWristIfBelowDown() {
    // Raise wrist if below loadCargo
    addSequential(new ConditionalCommand(new WristMoveToAngle(WristAngle.down)){
      @Override
      protected boolean condition() {
        return Robot.wrist.getWristAngle() < Robot.robotPrefs.wristDown - 5.0;
      }
    });
  }
}
