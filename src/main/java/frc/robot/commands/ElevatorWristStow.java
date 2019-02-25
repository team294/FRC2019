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

public class ElevatorWristStow extends CommandGroup {
  /**
   * Sequence to stow the wrist, regardless of starting angle.
   */
  public ElevatorWristStow() {
    addSequential(new ConditionalCommand(new ElevatorWristStowA(), new ElevatorWristStowB()){
      @Override
      protected boolean condition() {
        if (Robot.log.getLogLevel() <= 2) {
          Robot.log.writeLog("ElevatorWristStow", "Check 1", "Wrist angle," + Robot.wrist.getWristAngle()
            + ",Climb angle," + Robot.climb.getClimbAngle());
        }
        return Robot.wrist.getWristAngle() <= Robot.robotPrefs.wristKeepOut;
      }
    });
  }
}
