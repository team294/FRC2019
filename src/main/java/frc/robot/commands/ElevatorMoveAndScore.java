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

public class ElevatorMoveAndScore extends CommandGroup {
  /**
   * Add your docs here.
   */
  public ElevatorMoveAndScore(RobotPreferences.ElevatorPosition position) {
    addSequential(new WristMoveToAngle(WristAngle.straight));
    addSequential(new ElevatorMoveToLevel(position));
    addSequential(new ConditionalCommand(new WristMoveToAngle(WristAngle.up)) {
      @Override
      protected boolean condition() {
        if (Robot.cargo.getPhotoSwitch() && position == ElevatorPosition.hatchHigh) {
          return true;
        } else {
          return false;
        }
      }
    });
    addSequential(new CargoOuttakeOrHatchManipulate());
  }
}
