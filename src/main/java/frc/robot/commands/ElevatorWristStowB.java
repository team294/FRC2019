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
import frc.robot.utilities.RobotPreferences.WristAngle;

public class ElevatorWristStowB extends CommandGroup {
  /**
   * Sequence to stow the wrist.
   * <p>NOTE:  Only run this if the wrist is not already stowed or in the keepout region.
   */
  public ElevatorWristStowB() {
    // Start moving climber if we need to deploy the wrist and the climber is in the way
    addSequential(new ConditionalCommand(new ClimbArmSetAngle(Robot.robotPrefs.climbWristMovingSafe)){
      @Override
      protected boolean condition() {
        if (Robot.log.getLogLevel() <= 2) {
          Robot.log.writeLog("ElevatorWristStowB", "Check 1", "Wrist angle," + Robot.wrist.getWristAngle()
            + ",Climb angle," + Robot.climb.getClimbAngle());
        }
        return Robot.climb.getClimbAngle() > Robot.robotPrefs.climbWristMovingSafe;
      }
    });

    // Finish moving climber if we need to deploy the wrist and the climber is in the way
    addSequential(new ConditionalCommand(new WristMoveToAngle(WristAngle.stowed)){
      @Override
      protected boolean condition() {
        if (Robot.log.getLogLevel() <= 2) {
          Robot.log.writeLog("ElevatorWristStowB", "Check 2", "Wrist angle," + Robot.wrist.getWristAngle()
            + ",Elevator position," + Robot.elevator.getElevatorPos());
        }
        return Robot.elevator.getElevatorPos() <= Robot.robotPrefs.elevatorWristSafeStow;
      }
    });
  }
}
