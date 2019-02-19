/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class ClimbSequence extends CommandGroup {
  /**
   * Add your docs here.
   */
  public ClimbSequence() {
    // addParallel(new WristRaiseUntilStowed());
    addSequential(new ClimbMoveUntilVacuum(Robot.robotPrefs.climbVacuumAngle));
    addSequential(new ClimbLiftRobot(Robot.robotPrefs.climbLiftAngle));
  }
}
