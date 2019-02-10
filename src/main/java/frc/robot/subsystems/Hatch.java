/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.utilities.RobotPreferences;
import frc.robot.utilities.RobotPreferences.HatchPistonPositions;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Hatch extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private final DoubleSolenoid hatchPiston = new DoubleSolenoid(RobotMap.pneumaticHatchOut, RobotMap.pneumaticHatchIn);
  private HatchPistonPositions hatchPosition = HatchPistonPositions.Null;

  public Hatch() {
  }

  /**
	 * Sets the position of the hatch piston
	 * 
	 * @param position only accepts PistonPositions.Extended and PistonPositions.Retracted, other values are ignored
	 */
  public void setHatchPiston(RobotPreferences.HatchPistonPositions position) {
    if (position == RobotPreferences.HatchPistonPositions.Grab) {
      hatchPiston.set(Value.kForward);
      hatchPosition = HatchPistonPositions.Grab;
      SmartDashboard.putString("Disc Position", "Grab");
		}
		if (position == RobotPreferences.HatchPistonPositions.Release) {
      hatchPiston.set(Value.kReverse);
      hatchPosition = HatchPistonPositions.Release;
      SmartDashboard.putString("Disc Position", "Release");
    }
  }

  /**
	 * 
	 * @return position of hatch piston
	 */
	public HatchPistonPositions getHatchPiston() {
		return hatchPosition;
	}

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

}
