/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DigitalInput;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Climb extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private final Compressor compressor = new Compressor(0);
  private final WPI_TalonSRX climbMotor1 = new WPI_TalonSRX(RobotMap.climbMotor1);
  private final BaseMotorController climbMotor2 = new WPI_VictorSPX(RobotMap.climbMotor2);
  private final BaseMotorController climbVacuum = new WPI_VictorSPX(RobotMap.climbVacuum);
  private final DigitalInput vacuumSwitch = new DigitalInput(RobotMap.vacuumSwitch);
  private int periodicCount = 0;
  public double climbStartingPoint = 0;

  public Climb() {
    enableCompressor(true);

    climbMotor2.follow(climbMotor1);
    climbMotor1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
    zeroClimbEncoder();

    climbMotor2.setInverted(true);
    climbMotor1.clearStickyFaults(0);
    climbMotor2.clearStickyFaults(0);
    climbMotor1.setNeutralMode(NeutralMode.Brake);
    climbMotor2.setNeutralMode(NeutralMode.Brake);
  }

  /**
   * Enables or disables the compressor
   * @param turnOn true = turn on compressor when pressure drops
   * 				false = keep compressor off
   */
  public void enableCompressor(boolean turnOn) {
	  compressor.setClosedLoopControl(turnOn);
  }

  /**
   * Sets percent power of motor
   * @param percentPower between -1.0 and 1.0
   */
  public void setClimbMotorPercentOutput(double percentOutput) {
    climbMotor1.set(percentOutput);
  }

  /**
   * Turns on or turns off the vacuum
   * @param turnOn true turns vacuum on, false turns vacuum off
   */
  public void enableVacuum(boolean turnOn) {
    if (turnOn) {
      climbVacuum.set(ControlMode.PercentOutput, 0.5);
    }
    else {
      climbVacuum.set(ControlMode.PercentOutput, 0.0);
    }
  }

  /**
   * Sets current value of the climbEncoder as the new "zero"
   */
  public void zeroClimbEncoder() {
    climbStartingPoint = climbMotor1.getSelectedSensorPosition(0);
  }

  public double getClimbEncoderTicks() {
    return climbMotor1.getSelectedSensorPosition(0) - climbStartingPoint;
  }

  public double climbEncTicksToAngle (double encoderTicks) {
    return ((encoderTicks * 360) / RobotMap.encoderTicksPerRevolution);
  }

  public double climbAngleToEncTicks (double climbAngle) {
    return ((climbAngle * RobotMap.encoderTicksPerRevolution) / 360);
  }

  public double getClimbAngle() {
    return climbEncTicksToAngle(getClimbEncoderTicks());
  }

  public void stopClimbMotor() {
    climbMotor1.set(0.0);
  }

  public boolean isVacuumAchieved() {
    return vacuumSwitch.get();
  }

  /**
   * TODO
   * May need to put methods to find the Climb Encoder angle here
   * Depends on what they decide to do with the motors/encoders
   */

  public void updateClimbLog() {
    Robot.log.writeLog("Climb", "Update Variables", 
    "Climb1 Volts" + climbMotor1.getMotorOutputVoltage() + ",Climb2 Volts," + climbMotor2.getMotorOutputVoltage() + ",ClimbVac Volts," + climbVacuum.getMotorOutputVoltage() +
    ",Climb1 Amps," + Robot.pdp.getCurrent(RobotMap.climbMotor1) + ",Climb2 Amps," + Robot.pdp.getCurrent(RobotMap.climbMotor2) + ",ClimbVac Amps," + Robot.pdp.getCurrent(RobotMap.climbVacuum) +
    ",ClimbEnc Ticks," + getClimbEncoderTicks());
  }
  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  @Override
  public void periodic() {

    if (DriverStation.getInstance().isEnabled()) {
      if ((++periodicCount) >= 25) {
        updateClimbLog();
        periodicCount=0;  
      }
    }
  }
}
