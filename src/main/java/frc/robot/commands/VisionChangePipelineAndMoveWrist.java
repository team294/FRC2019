/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.utilities.RobotPreferences.ElevatorPosition;

public class VisionChangePipelineAndMoveWrist extends CommandGroup {

  /**
   * Changes vision pipeline.  If pipeline = 0 or 1, also moves wrist to vision position.
    * @param pipeline  0 or 1 = vision, 2 = driver feed)
   */
  public VisionChangePipelineAndMoveWrist(double pipeline) {
    addSequential(new VisionChangePipeline(pipeline));
    if (pipeline < 2) {
      addSequential(new ElevatorWristMoveAndPrepare(ElevatorPosition.vision));
    }
  }
}
