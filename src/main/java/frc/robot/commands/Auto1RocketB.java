/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;

public class Auto1RocketB extends CommandGroup {
  /**
   * Pathfinder path to go to back side of rocket starting from level 1.
   * @param fieldSide "Left" or "Right"
   */
  public Auto1RocketB(String fieldSide) {
    addSequential(new DrivePathfinder(fieldSide + "1RocketB", true, false));
    addSequential(new ConditionalCommand(new TurnWithGyro(-90, false), new TurnWithGyro(90, false)){
    
      @Override
      protected boolean condition() {
        if (fieldSide.equals("Left")) return true;
        else return false;
      }
    });

    addSequential(new DrivePathfinder(fieldSide + "1RocketB", false, true));
  }
}
