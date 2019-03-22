/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class ClimbSequence extends CommandGroup {
  /**
   * Climbing sequence! Retracts the rear hatch mechanism,
   * stows the wrist safely (if needed), moves arm to get vacuum,
   * then lifts the robot.
   */
  public ClimbSequence() {
    // addParallel(new RearHatchSet(false));
    addParallel(new ElevatorWristStow());
    addSequential(new ClimbMoveUntilVacuum(Robot.robotPrefs.climbVacuumAngle));
    // addSequential(new ElevatorMoveToLevel(Robot.robotPrefs.elevatorBottomToFloor));
    addSequential(new ClimbArmSetAngle(Robot.robotPrefs.climbLiftAngle));
  }
}
