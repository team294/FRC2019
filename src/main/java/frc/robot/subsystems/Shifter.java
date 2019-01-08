/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Shifter extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  //private final Solenoid shifter = new Solenoid(RobotMap.pnuematicShifter);
  private final DoubleSolenoid shifter = new DoubleSolenoid(RobotMap.pnuematicShifterLow, RobotMap.pnuematicShifterHigh);
  

  public Shifter() {
		super();
	}

	/**
	 * Shifts according to parameter
	 * 
	 * @param high
	 *            true for high gear, false for low gear
	 */
	public void setShift(boolean high) {
		// shifter.set(high ? false : true); // shifter true is high gear, shifter false is low gear
		shifter.set(high ? Value.kForward : Value.kReverse); // shifter Forward is high gear, shifter Reverse is low gear
	}

	/**
	 * Returns the state of the shifter
	 * 
	 * @return true for high gear, false for low
	 */
	public boolean isShifterInHighGear() {
		// return shifter.get() == false; // get() false is high gear, get() true is low gear
		return shifter.get() == Value.kForward; // get() Forward is high gear, get() Reverse is low gear
	}

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
