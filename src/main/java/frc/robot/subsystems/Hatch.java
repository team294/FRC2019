/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import frc.robot.Robot;
import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.Solenoid;

public class Hatch extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private final Solenoid hatchPiston = new Solenoid(RobotMap.pneumaticHatchIntake);

  public Hatch() {
  }

  // hatch panel is not secured
  public void openHatch() {
    hatchPiston.set(true);
  }

  // hatch panel is secured
  public void closeHatch() {
    hatchPiston.set(false);
  }

  public boolean getHatchPistonPosition() {
    return hatchPiston.get();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
