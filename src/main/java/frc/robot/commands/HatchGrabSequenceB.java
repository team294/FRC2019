/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class HatchGrabSequenceB extends CommandGroup {
  /**
   * set hatch grabber to grabbed and retract extension
   */
  public HatchGrabSequenceB() {
    addSequential(new HatchFingersGrab(true));
    addSequential(new WaitCommand(0.1));
    addSequential(new HatchExtensionExtend(false));
  }
}
