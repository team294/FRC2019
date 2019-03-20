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

import edu.wpi.first.wpilibj.command.Subsystem;

import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * New hatch intake subsystem ("973 intake")
 */
public class NewHatch extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private final WPI_TalonSRX hatchMotor = new WPI_TalonSRX(RobotMap.hatchMotor);

  public NewHatch() {
    hatchMotor.set(ControlMode.PercentOutput, 0);
    hatchMotor.setNeutralMode(NeutralMode.Coast);
    hatchMotor.enableVoltageCompensation(true);
    hatchMotor.setInverted(false); // TODO determine if inverted
    }

  /**
   * Sets the percent output of the hatch motor
   * 
   * @param percent percent output
   */
  public void setHatchMotorPercentOutput(double percent) {
    hatchMotor.set(ControlMode.PercentOutput, percent);
    Robot.log.writeLog("Hatch", "Hatch Motor", "Percent Output" + percent);
  }

  /**
   * Stop the hatch intake motor
   */
  public void stopHatchIntake() {
    setHatchMotorPercentOutput(0);
    Robot.log.writeLog("Hatch", "Motor", "Stop Hatch");
  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
