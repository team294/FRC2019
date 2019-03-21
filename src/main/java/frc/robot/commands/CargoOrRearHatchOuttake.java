/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import frc.robot.Robot;

public class CargoOrRearHatchOuttake extends CommandGroup {
  /**
   * If cargo photo switch is triggered, run cargo motor to outtake.
   * Otherwise, run rear hatch motor to outtake.
   */
  public CargoOrRearHatchOuttake() {
    addSequential(new ConditionalCommand(new CargoOuttake(-0.8), new RearHatchOuttake(-0.6)) {
      @Override
      protected boolean condition() {
        return Robot.cargo.getPhotoSwitch();
      }
    });
  }
}
