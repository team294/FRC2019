/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.utilities.FileLog;

public class Cargo extends Subsystem {
  private final BaseMotorController cargoMotor1 = new WPI_VictorSPX(RobotMap.cargoMotor1); // top motor
  // private final BaseMotorController cargoMotor2 = new WPI_VictorSPX(RobotMap.cargoMotor2); // bottom motor
  private final DigitalInput photoSwitch = new DigitalInput(RobotMap.photoSwitchCargo); // Cargo Sensor

  public Cargo() {
    cargoMotor1.set(ControlMode.PercentOutput, 0);
    cargoMotor1.setNeutralMode(NeutralMode.Brake);
    // cargoMotor1.configVoltageCompSaturation(11.0, 0);
    // cargoMotor1.enableVoltageCompensation(true);
    cargoMotor1.setInverted(true);

    /*
    cargoMotor2.set(ControlMode.PercentOutput, 0);
    cargoMotor2.setNeutralMode(NeutralMode.Brake);
    cargoMotor2.configVoltageCompSaturation(11.0, 0);
    cargoMotor2.enableVoltageCompensation(true);
    cargoMotor2.setInverted(true);
    */

  } 

  /**
   * Set the speed of the cargo motors
   * @param percent From -1 (full speed out) to +1 (full speed in)
   */
  public void setCargoMotorPercentOutput(double percent) {
    cargoMotor1.set(ControlMode.PercentOutput, percent); 

    Robot.log.writeLog("Cargo", "Percent Power", "Percent Power," + percent);
  }

  /**
   * Stop the cargo intake motors
   */
  public void stopCargoIntake() {
    setCargoMotorPercentOutput(0.0);
  }

  public boolean getPhotoSwitch(){
    if(Robot.log.getLogLevel() <= 2){
      Robot.log.writeLog("Cargo", "Photo Sensor", "Photo Sensor," + photoSwitch.get());
    }
    return !photoSwitch.get();
  }

  /**
   * Reads the photo switch to see if we have a ball
   * @return true = has ball, false = does not have ball
   */
  public boolean hasBall(){
    if(Robot.log.getLogLevel() <= 2){
      Robot.log.writeLog("Cargo", "Photo Sensor", "Photo Sensor," + photoSwitch.get());
    }
    return !photoSwitch.get();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  @Override
  public void periodic() {

		if (Robot.log.getLogRotation() == FileLog.CARGO_CYCLE) {
      SmartDashboard.putBoolean("Cargo Has Ball", hasBall());

      Robot.log.writeLog(false, "Cargo", "Update Variables", "Photo Switch," + hasBall() + 
        ",Volt," + cargoMotor1.getMotorOutputVoltage() + ",Amp," + Robot.pdp.getCurrent(RobotMap.cargoMotor1PDP));
    }

  }

}
