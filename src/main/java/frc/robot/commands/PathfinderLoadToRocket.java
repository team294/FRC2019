/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class PathfinderLoadToRocket extends CommandGroup {
  /**
   * Add your docs here.
   */
  public PathfinderLoadToRocket() {
    addSequential(new DrivePathfinder("RLoadToRocketPT1-A", true, false));
    // addSequential(new TurnGyro(90)); //TODO add TurnGyro command
    addSequential(new DrivePathfinder("RLoadToRocketPT2-A2", false, true));
  }
}