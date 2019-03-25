/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class HatchFrontAndRearSet extends CommandGroup {
  /**
   * Grab or releae a hatch.  This sequence activates both the
   * front grabber (on the cargo intake) and the rear grabber (on the climber).
   * It also extends the rear grabber.
   * @param grab true = grab a hatch, false = release the hatch
   */
  public HatchFrontAndRearSet(boolean grab) {
    // Move the front grabber as needed.
    addParallel(new HatchSet(grab));

    // Deploy and turn on the rear grabber as needed.
    addSequential(new RearHatchSet(true));
    if (grab) {
      addSequential(new RearHatchSetPercentOutput(0.6, 0));
    } else {
      addSequential(new RearHatchSetPercentOutput(-0.6, 0));
    }
  }
}
