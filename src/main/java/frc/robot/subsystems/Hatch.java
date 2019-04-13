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
import edu.wpi.first.wpilibj.Solenoid;


public class Hatch extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private final Solenoid hatchPiston = new Solenoid(RobotMap.pneumaticHatch);
  private final Solenoid hatchExtensionPiston = new Solenoid(RobotMap.hatchExtensionPiston);
  //private HatchPistonPositions hatchPosition = HatchPistonPositions.unknown;
  private boolean hatchPosition = true;
  private boolean hatchExtensionPosition = true;

  public Hatch() {
      setHatchPiston(true);
      setHatchExtensionPiston(true);
  }

  /**
	 * Sets the position of the hatch piston
	 * 
	 * @param position true is grab hatch; false is release hatch
	 */
  public void setHatchPiston(boolean position) {
    if (position) {
      hatchPiston.set(false);
      hatchPosition = true;
      SmartDashboard.putString("Disc Position", "Grab");
      SmartDashboard.putBoolean("Disc Grabbed", hatchPosition);
      //Robot.leds.setColor(LedHandler.Color.GREEN, true); // blink the LEDs when the hatch grabber in engaged
		} else {
      hatchPiston.set(true);
      hatchPosition = false;
      SmartDashboard.putString("Disc Position", "Release");
      SmartDashboard.putBoolean("Disc Grabbed", hatchPosition);
    }
  }

  /**
	 * 
	 * @return position of hatch piston
	 */
	public boolean getHatchPiston() {
		return hatchPosition;
	}

  /**
	 * Sets the position of the hatch extension piston
	 * 
	 * @param position true is retract hatch grabber; false is extend hatch grabber
	 */
  public void setHatchExtensionPiston(boolean position) {
    if (position) {
      hatchExtensionPiston.set(false);
      hatchExtensionPosition = true;
      SmartDashboard.putString("Hatch Grabber Position", "Retract");
		} else {
      hatchExtensionPiston.set(true);
      hatchExtensionPosition = false;
      SmartDashboard.putString("Hatch Grabber Position", "Extend");
    }
  }

  /**
	 * 
	 * @return position of hatch extension piston
	 */
	public boolean getHatchExtensionPiston() {
		return hatchExtensionPosition;
	}

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

}
