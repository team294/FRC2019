/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.utilities.RobotPreferences.ElevatorPosition;
import frc.robot.utilities.RobotPreferences.WristAngle;

public class CargoIntakeFromGround extends CommandGroup {
  /**
   * Add your docs here.
   */
  public CargoIntakeFromGround() {
    addSequential(new WristMoveToAngle(WristAngle.straight));
    addSequential(new ElevatorMoveToLevel(Robot.robotPrefs.cargoGround));
    addSequential(new WristMoveToAngle(WristAngle.down));
    addSequential(new CargoIntake());
    addSequential(new ElevatorMoveToLevel(ElevatorPosition.hatchLow));
    addSequential(new WristMoveToAngle(WristAngle.straight));
  }
}
