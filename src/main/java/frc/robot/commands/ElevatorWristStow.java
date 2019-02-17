/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.utilities.RobotPreferences.ElevatorPosition;
import frc.robot.utilities.RobotPreferences.WristAngle;

public class ElevatorWristStow extends CommandGroup {
  /**
   * Add your docs here.
   */
  public ElevatorWristStow() {
    addSequential(new WristMoveToAngle(WristAngle.straight));
    addSequential(new ElevatorMoveToLevel(ElevatorPosition.bottom));
    addSequential(new WristMoveToAngle(WristAngle.stowed));
  }
}
