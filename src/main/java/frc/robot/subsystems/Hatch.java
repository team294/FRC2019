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
import frc.robot.utilities.RobotPreferences.PistonPositions;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Hatch extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private final DoubleSolenoid hatchPiston = new DoubleSolenoid(RobotMap.pneumaticHatchOut, RobotMap.pneumaticHatchIn);
  private PistonPositions hatchPosition = PistonPositions.Null;

  public Hatch() {
  }

  /**
	 * Sets the position of the hatch piston
	 * 
	 * @param position only accepts PistonPositions.Extended and PistonPositions.Retracted, other values are ignored
	 */
  public void setHatchPiston(RobotPreferences.PistonPositions position) {
    if (position == RobotPreferences.PistonPositions.Extended) {
      hatchPiston.set(Value.kForward);
      hatchPosition = PistonPositions.Extended;
      SmartDashboard.putString("Piston Position", "Extended");
		}
		if (position == RobotPreferences.PistonPositions.Retracted) {
      hatchPiston.set(Value.kReverse);
      hatchPosition = PistonPositions.Retracted;
      SmartDashboard.putString("Piston Position", "Retracted");
    }
  }

  /**
	 * 
	 * @return position of hatch piston
	 */
	public PistonPositions getHatchPiston() {
		return hatchPosition;
	}

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

}
