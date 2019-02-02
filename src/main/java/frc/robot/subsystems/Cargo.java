/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class Cargo extends Subsystem {
  private final TalonSRX cargoMotor1 = new TalonSRX(RobotMap.cargoMotor1); // top motor
  private final TalonSRX cargoMotor2 = new TalonSRX(RobotMap.cargoMotor2); // bottom motor

  public Cargo() {
    // TODO determine which motor to invert
    cargoMotor1.set(ControlMode.PercentOutput, 0);
    cargoMotor1.setNeutralMode(NeutralMode.Coast);
    cargoMotor1.enableVoltageCompensation(true);
    cargoMotor2.set(ControlMode.PercentOutput, 0);
    cargoMotor2.setNeutralMode(NeutralMode.Coast);
    cargoMotor2.enableVoltageCompensation(true);
  } 

  public void setCargoMotorPercent(double percent) {
    cargoMotor1.set(ControlMode.PercentOutput, percent); 
    cargoMotor2.set(ControlMode.PercentOutput, percent);
  }

  public void intakeCargo() {
    setCargoMotorPercent(0.3);
  }

  public void outtakeCargo() {
    setCargoMotorPercent(-0.3);
  }

  public void stopCargoIntake() {
    setCargoMotorPercent(0);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
