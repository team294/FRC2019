/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.utilities.FileLog;

/**
 * Rear hatch intake subsystem.
 */
public class RearHatch extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private final WPI_TalonSRX rearHatchMotor = new WPI_TalonSRX(RobotMap.rearHatchMotor);
  private final Solenoid rearHatchPiston = new Solenoid(RobotMap.pneumaticRearHatch);
  private boolean rearHatchPosition = false;

  public RearHatch() {
    rearHatchMotor.set(ControlMode.PercentOutput, 0);
    rearHatchMotor.setNeutralMode(NeutralMode.Brake);
    rearHatchMotor.configVoltageCompSaturation(12.0);
    rearHatchMotor.enableVoltageCompensation(true);
    rearHatchMotor.setInverted(false); // TODO determine if inverted
    setRearHatchPiston(false);
    }

  /**
   * Sets the percent output of the rear hatch intake motor (positive = intaking, negative = outtaking).
   * 
   * @param percent percent output
   */
  public void setRearHatchMotorPercentOutput(double percent) {
    rearHatchMotor.set(ControlMode.PercentOutput, percent);
    Robot.log.writeLog("Rear Hatch", "Motor", "Percent Output," + percent);
  }

  /**
   * Stop the rear hatch intake motor.
   */
  public void stopRearHatch() {
    setRearHatchMotorPercentOutput(0);
  }

  /**
	 * Sets the position of the hatch piston if the climber above the climb prep angle
   * (to avoid unintentionally extending during the climb).
	 * 
	 * @param extend true = extended position, false = retracted position
	 */
  public void setRearHatchPiston(boolean extend) {
    if (extend && Robot.climb.getClimbAngle() > Robot.robotPrefs.climbPrep - 5) {
      rearHatchPiston.set(true);
      rearHatchPosition = true;
      Robot.log.writeLog("Rear Hatch", "Piston Position", "Extended");
      SmartDashboard.putString("Rear Hatch Position", "Extended");
		} else {
      rearHatchPiston.set(false);
      rearHatchPosition = false;
      Robot.log.writeLog("Rear Hatch", "Piston Position", "Retracted");
      SmartDashboard.putString("Rear Hatch Position", "Retracted");
    }
  }

  /**
	 * 
	 * @return true = extended, false = retracted.
	 */
	public boolean isRearHatchPistonExtended() {
		return rearHatchPosition;
	}

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  @Override
  public void periodic() {

		if (Robot.log.getLogRotation() == FileLog.REARHATCH_CYCLE) {
      Robot.log.writeLog(false, "Rear Hatch", "Update Variables", 
        "Piston," + (isRearHatchPistonExtended() ? "Extended" : "Retracted") + 
        ",Volt," + rearHatchMotor.getMotorOutputVoltage() + ",Amp," + rearHatchMotor.getOutputCurrent());
    }

  }

}
