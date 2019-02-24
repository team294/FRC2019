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

public class ElevatorWristMoveAndPrepare extends CommandGroup {
  /**
   * Add your docs here.
   */
  public ElevatorWristMoveAndPrepare(RobotPreferences.ElevatorPosition position) {
    // Move climber if we need to deploy the wrist and the climber is in the way
    addSequential(new ConditionalCommand(new ClimbArmSetAngle(Robot.robotPrefs.climbWristMovingSafe)){
      @Override
      protected boolean condition() {
        return Robot.wrist.getWristAngle() > Robot.robotPrefs.wristKeepOut &&
          Robot.climb.getClimbAngle() > Robot.robotPrefs.climbWristMovingSafe;
      }
    });
    
    if (position == ElevatorPosition.bottom || position == ElevatorPosition.hatchLow || position == ElevatorPosition.wristStow) {
      // Raise wrist if below horizontal
      addSequential(new ConditionalCommand(new WristMoveToAngle(WristAngle.straight)){
        @Override
        protected boolean condition() {
          return Robot.wrist.getWristAngle() < Robot.robotPrefs.wristStraight - 5.0;
        }
      });
    } else if (position == ElevatorPosition.groundCargo) {
      // Raise wrist if below loadCargo
      addSequential(new ConditionalCommand(new WristMoveToAngle(WristAngle.down)){
        @Override
        protected boolean condition() {
          return Robot.wrist.getWristAngle() < Robot.robotPrefs.wristDown - 5.0;
        }
      });
    }
    addSequential(new ElevatorMoveToLevel(position));
    addParallel(new ConditionalCommand(new WristMoveToAngle(WristAngle.straight), new WristMoveToAngle(WristAngle.up)) {
      @Override
      protected boolean condition() {
        if (Robot.cargo.getPhotoSwitch() && position == ElevatorPosition.hatchHigh) {
          return true;
        } else {
          return false;
        }
      }
    });
  }
}
