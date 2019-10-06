/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutoRocketToLoad extends CommandGroup {
  /**
   * Add your docs here.
   */
  public AutoRocketToLoad() {
    addSequential(new HatchFingersGrab(false));
    addSequential(new DrivePathfinder("RightRocketLoadF1", false, false));
    addSequential(new TurnWithGyro(-180, false));
    addSequential(new DrivePathfinder("RightRocketLoadF2", true, true));
  }
}
