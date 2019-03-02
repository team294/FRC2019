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
import frc.robot.utilities.RobotPreferences.ElevatorPosition;
import frc.robot.utilities.RobotPreferences.WristAngle;

public class ElevatorWristStowA extends CommandGroup {
  /**
   * Sequence to stow the wrist.
   * <p>NOTE:  Only run this if the wrist is not already stowed or in the keepout region.
   */
  public ElevatorWristStowA() {
    // Start moving climber if we need to deploy the wrist and the climber is in the way
    addParallel(new ConditionalCommand(new ClimbArmSetAngle(Robot.robotPrefs.climbWristMovingSafe - 5.0)){
      @Override
      protected boolean condition() {
        Robot.log.writeLog("ElevatorWristStowA", "Check 1", "Wrist angle," + Robot.wrist.getWristAngle()
          + ",Climb angle," + Robot.climb.getClimbAngle());
        return Robot.climb.getClimbAngle() > Robot.robotPrefs.climbWristMovingSafe;
      }
    });

    addSequential(new ConditionalCommand(new WristMoveToAngle(WristAngle.straight)){
      @Override
      protected boolean condition() {
        Robot.log.writeLog("ElevatorWristStowA", "Check 2", "Wrist angle," + Robot.wrist.getWristAngle()
          + ",Climb angle," + Robot.climb.getClimbAngle());
        return Robot.wrist.getWristAngle() < Robot.robotPrefs.wristStraight - 2.0;
      }
    });

    addParallel(new WristMoveToAngle(Robot.robotPrefs.wristKeepOut - 5.0));
    addSequential(new ElevatorMoveToLevel(ElevatorPosition.bottom));

    // Finish moving climber if we need to deploy the wrist and the climber is in the way
    addSequential(new ConditionalCommand(new ClimbArmSetAngle(Robot.robotPrefs.climbWristMovingSafe - 5.0)){
      @Override
      protected boolean condition() {
        Robot.log.writeLog("ElevatorWristStowA", "Check 3", "Wrist angle," + Robot.wrist.getWristAngle()
          + ",Climb angle," + Robot.climb.getClimbAngle());
        return Robot.climb.getClimbAngle() > Robot.robotPrefs.climbWristMovingSafe;
      }
    });

    addSequential(new WristMoveToAngle(WristAngle.stowed));
  }
}
