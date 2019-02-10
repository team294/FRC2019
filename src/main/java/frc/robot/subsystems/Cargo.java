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
import edu.wpi.first.wpilibj.DigitalInput;


import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class Cargo extends Subsystem {
  private final WPI_TalonSRX cargoMotor1 = new WPI_TalonSRX(RobotMap.cargoMotor1); // top motor
  private final WPI_TalonSRX cargoMotor2 = new WPI_TalonSRX(RobotMap.cargoMotor2); // bottom motor
  private final DigitalInput photoSwitch = new DigitalInput(RobotMap.photoSwitchCargo);

  public Cargo() {
    // TODO determine which motor to invert
    cargoMotor1.set(ControlMode.PercentOutput, 0);
    cargoMotor1.setNeutralMode(NeutralMode.Coast);
    cargoMotor1.configVoltageCompSaturation(11.0, 0);
    cargoMotor1.enableVoltageCompensation(true);
    cargoMotor1.setInverted(false);

    cargoMotor2.set(ControlMode.PercentOutput, 0);
    cargoMotor2.setNeutralMode(NeutralMode.Coast);
    cargoMotor2.configVoltageCompSaturation(11.0, 0);
    cargoMotor2.enableVoltageCompensation(true);
    cargoMotor2.setInverted(true);

  } 

  /**
   * Set the speed of the cargo motors
   * @param percent1 From -1 (full speed out) to +1 (full speed in)
   * @param percent2 From -1 (full speed out) to +1 (full speed in)
   */
  public void setCargoMotorPercent(double percent1, double percent2) {
    cargoMotor1.set(ControlMode.PercentOutput, percent1); 
    cargoMotor2.set(ControlMode.PercentOutput, percent2);
    if(Robot.log.getLogLevel() <= 2){
      Robot.log.writeLog("Cargo", "Percent Power", ",Percent Power Top," + percent1 + ",Percent Power Bot," + percent2);
    }
  }

  public boolean getPhotoSwitch(){
    if(Robot.log.getLogLevel() <= 2){
      Robot.log.writeLog("Cargo", "Photo Sensor", ",Photo Sensor," + photoSwitch.get());
    }
    return photoSwitch.get();
  }

  //TODO Change percent power when we get a cargo intake
  public void intakeCargo() {
    setCargoMotorPercent(0.5, 0.3);
    if(Robot.log.getLogLevel() <= 2){
      Robot.log.writeLog("Cargo", "Intake Cargo", "");
    }
  }

  public void outtakeCargo() {
    setCargoMotorPercent(-0.3, -0.3);
    if(Robot.log.getLogLevel() <= 2){
      Robot.log.writeLog("Cargo", "Outtake Cargo", "");
    }
  }

  public void stopCargoIntake() {
    setCargoMotorPercent(0, 0);
    if(Robot.log.getLogLevel() == 1){
      Robot.log.writeLog("Cargo", "Stop Cargo", "");
    }
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
