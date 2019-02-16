/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.SensorCollection;

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
  private final WPI_TalonSRX climbMotor2 = new WPI_TalonSRX(RobotMap.climbMotor2);
  private final BaseMotorController climbVacuum = new WPI_VictorSPX(RobotMap.climbVacuum1); //left Vacuum system
  //private final BaseMotorController climbVacuum2 = new WPI_VictorSPX(RobotMap.climbVacuum2); //right Vacuum system
  //private final DigitalInput vacuumSwitch = new DigitalInput(RobotMap.vacuumSwitch);
  private final SensorCollection climbLimit;
  private int periodicCount = 0;
  private int vacuumAchievedCount = 0; //increments every cycle vacuum is achieved

  private double rampRate = .005;
  private double kP = 1;
  private double kI = 0;
  private double kD = 0;
  private double kFF = 0;
  private int kIz = 0;
  private double kMaxOutput = 0.3;	   // TODO change to 1.0 after testing for max speed
  private double kMinOutput = -0.3;    // TODO change to -1.0 after testing for max speed

  public Climb() {
    enableCompressor(true);

    climbMotor1.follow(climbMotor2);
    climbMotor2.set(ControlMode.PercentOutput, 0);
    climbMotor2.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
    climbMotor2.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    climbLimit = climbMotor2.getSensorCollection();

    climbMotor2.config_kP(0, kP);
    climbMotor2.config_kI(0, kI);
    climbMotor2.config_kD(0, kD);
    climbMotor2.config_kF(0, kFF);
    climbMotor2.config_IntegralZone(0, kIz);
    climbMotor2.configClosedloopRamp(rampRate);
    climbMotor2.configPeakOutputForward(kMaxOutput);
    climbMotor2.configPeakOutputReverse(kMinOutput);

    climbMotor1.clearStickyFaults(0);
    climbMotor2.clearStickyFaults(0);
    climbVacuum.clearStickyFaults();
    //climbVacuum2.clearStickyFaults();
    climbMotor1.setNeutralMode(NeutralMode.Brake);
    climbMotor2.setNeutralMode(NeutralMode.Brake);
    climbVacuum.setNeutralMode(NeutralMode.Brake);
    //climbVacuum2.setNeutralMode(NeutralMode.Brake);

    Robot.robotPrefs.adjustClimbCalZero();
  }

  /**
   * Enables or disables the compressor
   * @param turnOn true = turn on compressor when pressure drops
   * 				      false = keep compressor off
   */
  public void enableCompressor(boolean turnOn) {
	  compressor.setClosedLoopControl(turnOn);
  }

  /**
   * Sets percent power of climb motors
   * @param percentPower between -1.0 (down full speed) and 1.0 (up full speed)
   */
  public void setClimbMotorPercentOutput(double percentOutput) {
    // Invert due to sign convention
    percentOutput = -percentOutput;
    // Cap to max power output allowed
    percentOutput = (percentOutput>kMaxOutput ? kMaxOutput : percentOutput);
    percentOutput = (percentOutput<kMinOutput ? kMinOutput : percentOutput);  

    climbMotor2.set(ControlMode.PercentOutput, percentOutput);
  }

  /**
   * Stops the climb motors
   */
  public void stopClimbMotor() {
    setClimbMotorPercentOutput(0.0);
  }

  /**
   * Sets angle of climb motors
   * @param angle target angle, in degrees (0 = horizontal behind robot, + = up, - = down)
   */
  public void setClimbPos(double angle) {
    if (Robot.robotPrefs.climbCalibrated) {
      climbMotor2.set(ControlMode.Position, climbAngleToEncTicks(angle) + Robot.robotPrefs.climbCalZero);
    }
  }

  /**
   * Turns on or turns off the vacuum
   * @param turnOn true turns vacuum on, false turns vacuum off
   */
  public void enableVacuum(boolean turnOn) {
    if (turnOn) {
      climbVacuum.set(ControlMode.PercentOutput, 0.5);
      //climbVacuum2.set(ControlMode.PercentOutput, 0.5);
    }
    else {
      climbVacuum.set(ControlMode.PercentOutput, 0.0);
      //climbVacuum2.set(ControlMode.PercentOutput, 0.0);
    }
  }

  /**
   * Sets current value of the climbEncoder as the new "zero"
   */
  public void zeroClimbEnc() {
    Robot.robotPrefs.setClimbCalibration(getClimbEncTicksRaw(), false);
  }

  /**
   * Calibrates the climb encoder, assuming we know the climber's current angle
   * @param angle current angle that the arm is physically at, in degrees (0 = horizontal behind robot, + = up, - = down)
   * @param saveToPrefs true = save the calibration to RobotPreferences
   */
  public void calibrateClimbEnc(double angle, boolean saveToPrefs) {
    Robot.robotPrefs.setClimbCalibration(getClimbEncTicksRaw() - climbAngleToEncTicks(angle), saveToPrefs);
  }  

  /**
   * Reads the climb encoder in raw ticks.
   * @return raw climb encoder value, in ticks
   */
  public double getClimbEncTicksRaw() {
    return climbMotor2.getSelectedSensorPosition(0);
  }

  /**
   * Reads the climb encoder in calibrated ticks (0 = horizontal, + = ??).
   * Note that the raw encoder value may wrap.
   * @return encoder value from horizontal
   */
  public double getClimbEncTicks() {
    return getClimbEncTicksRaw() - Robot.robotPrefs.climbCalZero;
  }

  /**
   * Convert encoder ticks to angle in degrees
   * @param encoderTicks use getClimbEncoderTicks()
   */
  public double climbEncTicksToAngle (double encoderTicks) {
    return ((encoderTicks * 360) / Robot.robotPrefs.encoderTicksPerRevolution);
  }

  /**
   * Convert angle in degrees to encoder ticks
   * @param climbAngle in degrees
   */
  public double climbAngleToEncTicks (double climbAngle) {
    return ((climbAngle * Robot.robotPrefs.encoderTicksPerRevolution) / 360);
  }

  /**
   * @return angle in degrees
   */
  public double getClimbAngle() {
    return climbEncTicksToAngle(getClimbEncTicks());
  }

  /**
   * @return true = vaccum is at the required pressure
   *          false = vacuum is not at the required pressure yet
   */
  public boolean isVacuumAchieved() {
    if(Robot.pdp.getCurrent(RobotMap.climbVacuum1PDP) > Robot.robotPrefs.vacuumCurrentThreshold) {
     //|| Robot.pdp.getCurrent(RobotMap.climbVacuum2PDP) > Robot.robotPrefs.rightVacuumCurrentThreshold) {
       vacuumAchievedCount++;
     } else {
       vacuumAchievedCount = 0;
     }
    return vacuumAchievedCount >= 5; // TODO change once current threshold is known
  }

  /**
   * @return true = climb is at its calibration angle
   */
  public boolean isClimbAtLimitSwitch() {
    return climbLimit.isFwdLimitSwitchClosed();
  }

  public void updateClimbLog() {
    Robot.log.writeLog("Climb", "Update Variables", 
    "Volts1" + climbMotor2.getMotorOutputVoltage() + ",Volts2," + climbMotor1.getMotorOutputVoltage() + 
    ",VacVolts," + climbVacuum.getMotorOutputVoltage() + //",VacVolts2," + climbVacuum2.getMotorOutputVoltage() +
    ",Amps1," + Robot.pdp.getCurrent(RobotMap.climbMotor2PDP) + ",Amps2," + Robot.pdp.getCurrent(RobotMap.climbMotor1PDP) + 
    ",VacAmps," + Robot.pdp.getCurrent(RobotMap.climbVacuum1PDP) + //",VacAmps2," + Robot.pdp.getCurrent(RobotMap.climbVacuum2PDP) +
    ",Enc Ticks," + getClimbEncTicks() + ",Enc Ang," + getClimbAngle());
  }
  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Climb calibrated", Robot.robotPrefs.climbCalibrated);
    SmartDashboard.putBoolean("Climb limit switch", isClimbAtLimitSwitch());
    SmartDashboard.putNumber("Climb angle", getClimbAngle());
    SmartDashboard.putNumber("Climb enc raw", getClimbEncTicksRaw());
    
    if (!Robot.robotPrefs.climbCalibrated || Robot.beforeFirstEnable) {
      if (isClimbAtLimitSwitch()) {
        calibrateClimbEnc(Robot.robotPrefs.climbStartingAngle, false);
      }
    }
    if (getClimbAngle() > Robot.robotPrefs.climbStartingAngle || getClimbAngle() < Robot.robotPrefs.climbMinAngle) {
      Robot.robotPrefs.setClimbUncalibrated();
    }
    if (DriverStation.getInstance().isEnabled()) {
      if ((++periodicCount) >= 10) {
        updateClimbLog();
        periodicCount=0;  
      }
    }
  }
}
