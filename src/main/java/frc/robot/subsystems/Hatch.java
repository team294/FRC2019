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
import edu.wpi.first.wpilibj.Solenoid;


public class Hatch extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private final Solenoid hatchPiston = new Solenoid(RobotMap.pneumaticHatchOut);
  //private HatchPistonPositions hatchPosition = HatchPistonPositions.unknown;
  private boolean hatchPosition = true;

  public Hatch() {
  }

  /**
	 * Sets the position of the hatch piston
	 * 
	 * @param position true is grab hatch; false is release hatch
	 */
  public void setHatchPiston(boolean position) {
    if (position == true) {
      hatchPiston.set(true);
      hatchPosition = true;
      SmartDashboard.putString("Disc Position", "Grab");
		}
		if (position == false) {
      hatchPiston.set(false);
      hatchPosition = false;
      SmartDashboard.putString("Disc Position", "Release");
    }
  }

  /**
	 * 
	 * @return position of hatch piston
	 */
	public boolean getHatchPiston() {
		return hatchPosition;
	}

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

}
