/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class ClimbPrepSequence extends CommandGroup {
  /**
   * Climbing prep sequence! Retracts the rear hatch mechanism,
   * stows the wrist safely (if needed), and moves arm to get vacuum.
   */
  public ClimbPrepSequence() {
    addSequential(new RearHatchSet(false));
    addSequential(new ElevatorWristStow());
    addSequential(new ClimbArmSetAngle(Robot.robotPrefs.climbPrep));
  }
}
