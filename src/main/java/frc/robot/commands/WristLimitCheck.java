/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.buttons.Trigger;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class WristLimitCheck extends Trigger {
  @Override
  public boolean get() {
    if(Robot.wrist.getWristUpperLimit() || Robot.wrist.getWristUpperLimit()) {
      return true;
    } else {
      return false;
    }  }
}
