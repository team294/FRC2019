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
   * Moves elevator and wrist as needed to go to specified position.
   */
  public ElevatorWristMoveAndPrepare(RobotPreferences.ElevatorPosition position) {
    // Move climber if we need to deploy the wrist and the climber is in the way
    addSequential(new ConditionalCommand(new ClimbArmSetAngle(Robot.robotPrefs.climbWristMovingSafe - 5.0)){
      @Override
      protected boolean condition() {
        if (Robot.log.getLogLevel() <= 3) {
          Robot.log.writeLog("ElevatorWristMove", "Check 1", "Wrist angle," + Robot.wrist.getWristAngle()
            + ",Climb angle," + Robot.climb.getClimbAngle());
        }
        return Robot.wrist.getWristAngle() > Robot.robotPrefs.wristKeepOut &&
          Robot.climb.getClimbAngle() > Robot.robotPrefs.climbWristMovingSafe;
      }
    });

    // Move wrist if it is stowed
    addSequential(new ConditionalCommand(new WristMoveToAngle(Robot.robotPrefs.wristKeepOut - 5.0)){
      @Override
      protected boolean condition() {
        if (Robot.log.getLogLevel() <= 3) {
          Robot.log.writeLog("ElevatorWristMove", "Check 2", "Wrist angle," + Robot.wrist.getWristAngle() );
        }
        return Robot.wrist.getWristAngle() > Robot.robotPrefs.wristKeepOut;
      }
    });

    // Raise wrist if below loadCargo
    addSequential(new ConditionalCommand(new WristMoveToAngle(WristAngle.down)){
      @Override
      protected boolean condition() {
        if (Robot.log.getLogLevel() <= 3) {
          Robot.log.writeLog("ElevatorWristMove", "Check 3", "Wrist angle," + Robot.wrist.getWristAngle() );
        }
        return Robot.wrist.getWristAngle() < Robot.robotPrefs.wristDown - 5.0;
      }
    });

    // If going to hatch high, determine if we have a ball to adjust wrist angle
    if (position == ElevatorPosition.hatchHigh) {
      addParallel(new ConditionalCommand(new WristMoveToAngle(WristAngle.up), new WristMoveToAngle(WristAngle.up)){
        @Override
        protected boolean condition() {
          if (Robot.log.getLogLevel() <= 3) {
            Robot.log.writeLog("ElevatorWristMove", "Check 4", "Has ball," + Robot.cargo.hasBall() );
          }
          return Robot.cargo.hasBall();
        }
      });
    } else if (position == ElevatorPosition.cargoShipCargo) {
      // If going to cargo ship shot, then use cargo ship angle
      addParallel(new WristMoveToAngle(WristAngle.cargoShot));
    } else if (position == ElevatorPosition.groundCargo){
      addParallel(new WristMoveToAngle(WristAngle.down));
      addParallel(new CargoIntake());
    } else {
      // Otherwise, move wrist straight
      addParallel(new WristMoveToAngle(WristAngle.straight));
    }

    addSequential(new ElevatorMoveToLevel(position));
    
    // If going to cargo ground intake, move wrist down last
    // if(position == ElevatorPosition.groundCargo){
    //   addSequential(new WristMoveToAngle(WristAngle.down));
    // }
  }
}
