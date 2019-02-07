/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.buttons.Trigger;
import frc.robot.Robot;
import frc.robot.subsystems.*;

/**
 * Get() returns true if the Wrist encoder has failed (but not the elevator encoder)
 */
public class CheckWristEncoderFail extends Trigger {
  @Override
  public boolean get() {
    if(!Robot.wrist.getEncOK()) {
      return true;
    } else {
      return false;
    }
  }
}
