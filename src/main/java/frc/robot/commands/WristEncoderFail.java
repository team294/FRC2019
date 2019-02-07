/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.RobotMap;

public class WristEncoderFail extends CommandGroup {
  /**
   * if wrist encoder fails, stop applying power to wrist until
   * elevator hits its bottom limit switch (automatic), and then apply power until
   * wrist hits its top limit switch
   */
  public WristEncoderFail() {
    addParallel(new WristOff());
    addSequential(new ElevatorMoveToLevel(RobotMap.StowHatch));
    addSequential(new WristApplyPowerUntilStowed());
  }
}
