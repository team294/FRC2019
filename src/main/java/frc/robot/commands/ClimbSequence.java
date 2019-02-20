/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;

public class ClimbSequence extends CommandGroup {
  /**
   * Add your docs here.
   */
  public ClimbSequence() {
    // If the climber is in the keepout zone, sart moving it to the safe zone so we can stow the wrist
    addParallel(new ConditionalCommand(new ClimbArmSetAngle(Robot.robotPrefs.climbWristMovingSafe)){
      @Override
      protected boolean condition() {
        return Robot.climb.getClimbAngle() > Robot.robotPrefs.climbWristMovingSafe;
      }
    });

    // TODO If the elevator is raised, then lower it (safely)
    // TODO If the wrist is not stowed, then stow it (safely)

    addSequential(new ClimbMoveUntilVacuum(Robot.robotPrefs.climbVacuumAngle));
    addSequential(new ClimbArmSetAngle(Robot.robotPrefs.climbLiftAngle));
  }
}
