/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.utilities.RobotPreferences.ElevatorPosition;

public class DriveAssist extends CommandGroup {
  /**
   * Pathfinder, Vision, and Line Following assisted driving (with gyro) to line up with scoring zone
   */
  public DriveAssist() {
    
    // addSequential(new VisionChangePipeline(0));
    addParallel(new Shift(false));    // go to low gear
    // addSequential(new ElevatorWristMoveAndPrepare(ElevatorPosition.hatchLow));
    addSequential(new ElevatorWristMoveAndPrepare(ElevatorPosition.vision));
    // addSequential(new WaitCommand(2.0));
    // addParallel(new ElevatorWristMoveAndPrepare(ElevatorPosition.hatchLow));
    addSequential(new DriveWithVision(false, false)); //  true, true  is endOnLine, use gyro
    addSequential(new ElevatorWristMoveAndPrepare(ElevatorPosition.hatchLow));
    // addSequential(new DriveWithLineFollowing(true));

  }
}
