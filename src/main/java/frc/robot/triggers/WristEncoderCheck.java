/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.triggers;

import edu.wpi.first.wpilibj.buttons.Trigger;
import frc.robot.Robot;

/**
 * Get() returns true if the Wrist encoder has failed (but not the elevator encoder)
 */
public class WristEncoderCheck extends Trigger {
  @Override
  public boolean get() {
    if(!Robot.wrist.isEncoderCalibrated()) {
      return true;
    } else {
      return false;
    }
  }
}
