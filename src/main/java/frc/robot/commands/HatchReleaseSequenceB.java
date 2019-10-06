/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class HatchReleaseSequenceB extends CommandGroup {
  /**
   * releases hatch, retract hatch extension, and sets hatch grabber to grabbed
   */
  public HatchReleaseSequenceB() {
    addSequential(new HatchFingersGrab(false));
    addSequential(new WaitCommand(0.1));
    addSequential(new HatchExtensionExtend(false));
    addSequential(new WaitCommand(3)); // was 1.5, changed for beach blitz
    addSequential(new HatchFingersGrab(true));
  }
}
