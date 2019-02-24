/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
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
import frc.robot.utilities.FileLog;

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
  private final DigitalInput vacuumSwitch = new DigitalInput(RobotMap.vacuumSwitch);
  private final AnalogInput analogVacuumSensor = new AnalogInput(RobotMap.analogVacuum);
  private final AnalogTrigger vacuumTrigger = new AnalogTrigger(analogVacuumSensor);
  private final SensorCollection climbLimit;

  private double rampRate = 0.5;
  private double kP = 2;
  private double kI = 0;
  private double kD = 0;
  private double kFF = 0;
  private int kIz = 0;
  private double kMaxOutput = 1.0;
  private double kMinOutput = -1.0;

  public Climb() {
    enableCompressor(true);

    climbMotor1.follow(climbMotor2);
    climbMotor2.set(ControlMode.PercentOutput, 0);
    climbMotor2.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
    climbMotor2.setSensorPhase(true);
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

    adjustClimbCalZero();

    // Oversampling for analog sensor
    analogVacuumSensor.setOversampleBits(4);
    analogVacuumSensor.setAverageBits(2);

    // Analog Trigger testing settings
    vacuumTrigger.setLimitsVoltage(0.5, 4.4); // Random boundaries, no idea what real values are
    //vacuumTrigger.setAveraged(true); // Use the averaged value instead of raw
    vacuumTrigger.setFiltered(true); // Use a 3-point filter to reject outliers. CANNOT BE USED WITH AVERAGE VALUE
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
    // percentOutput = -percentOutput;
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
    /* Can this version of the code get into a double-interlock scenario (wrist blocks climber and climber blocks wrist)?
    if (Robot.robotPrefs.climbCalibrated) {
      if ((getClimbAngle() < Robot.robotPrefs.climbWristMovingSafe && angle < Robot.robotPrefs.climbWristMovingSafe) ||
      (Robot.wrist.getWristAngle() < Robot.robotPrefs.wristKeepOut) || (Robot.wrist.getWristUpperLimit() && 
      (getClimbAngle() > Robot.robotPrefs.climbWristMovingSafe && getClimbAngle() < Robot.robotPrefs.climbWristStowedSafe) && angle < Robot.robotPrefs.climbWristMovingSafe)) {
        climbMotor2.set(ControlMode.Position, climbAngleToEncTicks(angle) + Robot.robotPrefs.climbCalZero);
        Robot.log.writeLog("Climb", "Set angle", "Angle," + angle + ",Interlock,OK");
      }
      else if ((Robot.wrist.getWristAngle() > Robot.robotPrefs.wristKeepOut) && (getClimbAngle() < Robot.robotPrefs.climbWristMovingSafe) && (angle > Robot.robotPrefs.climbWristMovingSafe)) {
        climbMotor2.set(ControlMode.Position, climbAngleToEncTicks(Robot.robotPrefs.climbWristMovingSafe) + Robot.robotPrefs.climbCalZero);
        Robot.log.writeLog("Climb", "Set angle", "Angle," + Robot.robotPrefs.climbWristMovingSafe + ",Interlock,Diverted");
      } else {
        Robot.log.writeLog("Climb", "Set angle", "Angle," + Robot.robotPrefs.climbWristMovingSafe + ",Interlock,Forbidden");
      }
    }
    */
    double safeAngle = angle;

    // Apply interlocks
    if (Robot.wrist.getWristUpperLimit() && Robot.wrist.getCurrentWristTarget() >= Robot.robotPrefs.wristStowed - 3) {
      // Wrist is stowed and is not being moved to another position.
      // Climber can move as far as climbWristStowedSafe
      safeAngle = (safeAngle > Robot.robotPrefs.climbWristStowedSafe) ? Robot.robotPrefs.climbWristStowedSafe : safeAngle;
    } else if (Robot.wrist.getWristAngle() > Robot.robotPrefs.wristKeepOut || Robot.wrist.getCurrentWristTarget() > Robot.robotPrefs.wristKeepOut) {
      // Wrist is in the keepout region or is moving to the keepout region.
      // Climber can move as far as climbWristMovingSafe
      safeAngle = (safeAngle > Robot.robotPrefs.climbWristMovingSafe) ? Robot.robotPrefs.climbWristMovingSafe : safeAngle;
    }

    climbMotor2.set(ControlMode.Position, climbAngleToEncTicks(safeAngle) + Robot.robotPrefs.climbCalZero);
    Robot.log.writeLog("Climb", "Set angle", "Desired angle," + angle + ",Set angle," + safeAngle);
  }

  /**
   * Turns on or turns off the vacuum
   * @param turnOn true turns vacuum on, false turns vacuum off
   */
  public void enableVacuum(boolean turnOn) {
    if (turnOn) {
      climbVacuum.set(ControlMode.PercentOutput, 1.0);
      //climbVacuum2.set(ControlMode.PercentOutput, 0.5);
    }
    else {
      climbVacuum.set(ControlMode.PercentOutput, 0.0);
      //climbVacuum2.set(ControlMode.PercentOutput, 0.0);
    }
  }

   /**
   * Returns true if pressure is low enough to initiate climb.
   * If switch is disconnected, isVacuumPresent reads false.
   * @return true = vacuum sufficient for climb, false = not enough vacuum
   */
  /*
  public boolean isVacuumPresent(){
    // Note:  Need to invert polarity from switch.
    return !vacuumSwitch.get();
  } */

  /**
   * Returns true if the pressure is low enough to climb. May have bad readings if sensor is disconnected (equates to ~25.5 reading)
   * @return true = pressure low enough, false = pressure too high
   */
  public boolean isVacuumPresent() {
    return getVacuumPressure(false) >= 20.0;
  }

  /**
   * Gets the value of the pressure gauge on the climb system according to the analog sensor
   * @param raw true for using raw data, false for using averaged data
   * @return pressure from 0 (atm) to 25.5 (upper limit of the sensor's sensitivity) inclusive
   */
  public double getVacuumPressure(boolean raw) {
    double out = (raw) ? analogVacuumSensor.getVoltage() * -5.7 + 27 : analogVacuumSensor.getAverageVoltage() * -5.7 + 27;
    return out;
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
	 * If the angle is reading >/< max/min angle, add/subtract 360 degrees to the climbCalZero accordingly
	 * <p> Note: when the motor is not inverted, upon booting up, an absolute encoder reads a value between 0 and 4096.
	 * 		 When the motor is inverted, upon booting up, an absolute encoder reads a value between 0 and -4096
	 * <p> Note: absolute encoder values don't wrap during operation
	 */
	public void adjustClimbCalZero() {
    Robot.log.writeLogEcho("Climb", "Adjust climb pre", "climb angle," + getClimbAngle() + 
      "raw ticks" + getClimbEncTicksRaw() + ",climbCalZero," + Robot.robotPrefs.climbCalZero);

		if(getClimbAngle() < Robot.robotPrefs.climbMinAngle - 10.0) {
      Robot.log.writeLogEcho("Climb", "Adjust climb", "Below min angle");
			Robot.robotPrefs.climbCalZero -= Robot.robotPrefs.encoderTicksPerRevolution;
		}
		else if(getClimbAngle() > Robot.robotPrefs.climbLimitAngle + 10.0) {
      Robot.log.writeLogEcho("Climb", "Adjust climb", "Above max angle");
			Robot.robotPrefs.climbCalZero += Robot.robotPrefs.encoderTicksPerRevolution;
    }
    
    Robot.log.writeLogEcho("Climb", "Adjust climb post", "climb angle," + getClimbAngle() + 
      "raw ticks" + getClimbEncTicksRaw() + ",climbCalZero," + Robot.robotPrefs.climbCalZero);
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
   * @return true = climb is at its calibration angle
   */
  public boolean isClimbAtLimitSwitch() {
    return climbLimit.isFwdLimitSwitchClosed();
  }

  public void updateClimbLog() {
    Robot.log.writeLog("Climb", "Update Variables", 
    "Volts1," + climbMotor2.getMotorOutputVoltage() + ",Volts2," + climbMotor1.getMotorOutputVoltage() + 
    ",VacVolts," + climbVacuum.getMotorOutputVoltage() + //",VacVolts2," + climbVacuum2.getMotorOutputVoltage() +
    ",Amps1," + Robot.pdp.getCurrent(RobotMap.climbMotor2PDP) + ",Amps2," + Robot.pdp.getCurrent(RobotMap.climbMotor1PDP) + 
    ",VacAmps," + Robot.pdp.getCurrent(RobotMap.climbVacuum1PDP) + //",VacAmps2," + Robot.pdp.getCurrent(RobotMap.climbVacuum2PDP) +
    ",EncCalZero," + Robot.robotPrefs.climbCalZero + ",Enc Raw," + getClimbEncTicksRaw() + ",Enc Ang," + getClimbAngle() + 
    ",Climb limit," + isClimbAtLimitSwitch() + ",Vacuum Achieved," + isVacuumPresent()
    );
  }
  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  
  @Override
  public void periodic() {
    if (Robot.log.getLogRotation() == FileLog.CLIMB_CYCLE) {
      SmartDashboard.putBoolean("Climb calibrated", Robot.robotPrefs.climbCalibrated);
      SmartDashboard.putBoolean("Climb limit switch", isClimbAtLimitSwitch());
      SmartDashboard.putNumber("Climb angle", getClimbAngle());
      SmartDashboard.putNumber("Climb enc raw", getClimbEncTicksRaw());
      SmartDashboard.putBoolean("Climb vacuum", isVacuumPresent());

      SmartDashboard.putNumber("Climb Analog Voltage", analogVacuumSensor.getVoltage());
      SmartDashboard.putNumber("Climb Analog Average (Oversampled) Voltage", analogVacuumSensor.getAverageVoltage());
      SmartDashboard.putBoolean("Vacuum Trigger In Window (0.5, 3.4)", vacuumTrigger.getInWindow());
      //SmartDashboard.putBoolean("Vacuum Trigger Rising/Falling", vacuumTrigger.getTriggerState());
      SmartDashboard.putNumber("Analog Vacuum Pressure", getVacuumPressure(false));

      if (DriverStation.getInstance().isEnabled()) {
        updateClimbLog(); 
      }
    }
    
    // Checks if the climb is not calibrated and automatically calibrates it once the reverse limit switch is pressed
    // If the climb isn't calibrated at the start of the match, then we can calibrate using manual climb control
    // or using the ClimbMoveToLimitThenCalibrate command.
    if (!Robot.robotPrefs.climbCalibrated ) {  // || Robot.beforeFirstEnable
      if (isClimbAtLimitSwitch()) {
        calibrateClimbEnc(Robot.robotPrefs.climbLimitAngle, false);
        updateClimbLog();
      }
    }
    
    // Un-calibrates the climb if the angle is outside of bounds.
    if (getClimbAngle() > Robot.robotPrefs.climbLimitAngle + 5.0 || getClimbAngle() < Robot.robotPrefs.climbMinAngle - 5.0) {
      Robot.robotPrefs.setClimbUncalibrated();
      updateClimbLog();
    }
  }
  
}
