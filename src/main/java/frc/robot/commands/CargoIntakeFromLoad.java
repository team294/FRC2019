/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.utilities.RobotPreferences.ElevatorPosition;
import frc.robot.utilities.RobotPreferences.WristAngle;

public class CargoIntakeFromLoad extends CommandGroup {
  /**
   * Moves wrist to 0, elevator to loading station intake position,
   * intakes cargo, and moves elevator to low cargo position
   */
  public CargoIntakeFromLoad() {
    addSequential(new WristMoveToAngle(WristAngle.straight));
    addSequential(new ElevatorMoveToLevel(ElevatorPosition.loadCargo));
    addSequential(new CargoIntake());
    addSequential(new ElevatorMoveToLevel(ElevatorPosition.hatchLow));
  }
}
