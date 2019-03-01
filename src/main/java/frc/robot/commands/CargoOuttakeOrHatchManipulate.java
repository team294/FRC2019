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

public class CargoOuttakeOrHatchManipulate extends CommandGroup {
  /**
   * If robot has cargo (photoswitch triggered) outtake cargo,
   * otherwise toggle hatch piston (release hatch) and back up
   */
  public CargoOuttakeOrHatchManipulate() {

    addSequential(new ConditionalCommand(new CargoOuttake(-0.8), new HatchScoreAndIntake()) {
      @Override
      protected boolean condition() {
        if (Robot.cargo.getPhotoSwitch()) {
          return true;
        } else {
          return false;
        }
      }
    });
  }
}
