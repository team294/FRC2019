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
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class TestHatch extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private final WPI_TalonSRX hatchMotor = new WPI_TalonSRX(RobotMap.testHatchMotor);

  public TestHatch(){
    hatchMotor.set(ControlMode.PercentOutput, 0);
    hatchMotor.setNeutralMode(NeutralMode.Coast);
    // hatchMotor.configVoltageCompSaturation(11.0, 0);
    hatchMotor.enableVoltageCompensation(true);
    hatchMotor.setInverted(false);
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void setPercentOutput(double percentOutput){
    hatchMotor.set(ControlMode.PercentOutput, percentOutput);
  }
  
  public void stopMotors(){
    setPercentOutput(0.0);
  }
}
