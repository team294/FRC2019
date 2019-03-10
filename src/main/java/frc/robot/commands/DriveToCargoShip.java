/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class DriveToCargoShip extends CommandGroup {
  /**
   * Drives to front face of cargo ship from being
   * lined up directly on the HAB platform
   */
  public DriveToCargoShip() {
    // addParallel(new ElevatorWristMoveAndPrepare(ElevatorPosition.hatchLow));
    addSequential(new DriveStraightDistanceProfile(100, 0, 80, 65));
    addSequential(new DriveAssist());
  }
}
