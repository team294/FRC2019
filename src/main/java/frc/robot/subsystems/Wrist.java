/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.*;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.RobotPreferences;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Wrist extends Subsystem {
  private WPI_TalonSRX wristMotor = new WPI_TalonSRX(RobotMap.wristMotor);
  private SensorCollection wristLimits;

	private int posMoveCount = 0; // increments every cycle the wrist moves up
	private int negMoveCount = 0; // increments every cycle the wrist moves down
	private double currEnc = 0.0; // current recorded encoder value
	private double encSnapShot = 0.0; // snapshot of encoder value used to make sure encoder is working
  private double encoderDegreesPerTicks = 360.0 / Robot.robotPrefs.encoderTicksPerRevolution;
  private double encoderTicksPerDegrees = Robot.robotPrefs.encoderTicksPerRevolution / 360.0;

  // TODO test PID terms with actual wrist
  private double kP = 0;
	private double kI = 0;
	private double kD = 0;
  private double kFF = 0;
  private int kIz = 0;
  private double kMaxOutput = 0.3; // up max output TODO increase after initial testing
  private double kMinOutput = -0.3; // down max output  TODO increase after initial testing
  private double rampRate = 0.5;

  public Wrist() {
    wristMotor.set(ControlMode.PercentOutput, 0);
    wristMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
    wristMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    wristMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    wristMotor.setNeutralMode(NeutralMode.Brake);
    wristMotor.clearStickyFaults();

    wristMotor.config_kP(0, kP);
		wristMotor.config_kI(0, kI);
		wristMotor.config_kD(0, kD);
		wristMotor.config_kF(0, kFF);
		wristMotor.config_IntegralZone(0, kIz);
		wristMotor.configClosedloopRamp(rampRate);
		wristMotor.configPeakOutputForward(kMaxOutput);
    wristMotor.configPeakOutputReverse(kMinOutput);
    
    wristLimits = wristMotor.getSensorCollection();

    adjustWristCalZero();
  }

  /**
   * Sets percent power of wrist motor
   * @param percentPower between -1.0 (down full speed) and 1.0 (up full speed)
   */
  public void setWristMotorPercentOutput(double percentOutput) {
    percentOutput = (percentOutput>kMaxOutput ? kMaxOutput : percentOutput);
    percentOutput = (percentOutput<kMinOutput ? kMinOutput : percentOutput);  

    if (Robot.log.getLogLevel() == 1) {
      Robot.log.writeLog("Wrist" , "Percent Output", "Percent Output," + percentOutput);
    }
    wristMotor.set(ControlMode.PercentOutput, percentOutput);
  }

  /**
   * Stops wrist motor
   */
  public void stopWrist() {
    setWristMotorPercentOutput(0.0);
  }

  /**
   * Only works when encoder is working and calibrated
   * If setting to greater than wristKeepOut, elevator position must be zero
   * and target must be less than 1 inch above starting position
   * @param angle target angle, in degrees (0 = horizontal in front of robot, + = up, - = down)
   */
  public void setWristAngle(double angle) {
    if (Robot.robotPrefs.wristCalibrated) {
      // TODO determine max degrees and elevator height tolerance
      if ( (angle < RobotPreferences.wristKeepOut && getWristAngle() < RobotPreferences.wristKeepOut) || 
          (Robot.elevator.getElevatorLowerLimit() && 
           Robot.elevator.getCurrentElevatorTarget() < (Robot.robotPrefs.elevatorBottomToFloor+1))) {
        wristMotor.set(ControlMode.Position, degreesToEncoderTicks(angle) + Robot.robotPrefs.wristCalZero);
        Robot.log.writeLog("Wrist", "Set angle", "Angle," + angle + ",Interlock,OK");
      } else {
        Robot.log.writeLog("Wrist", "Set angle", "Angle," + angle + ",Interlock,Forbidden");
      }
    }
  }

  /**
   * Calibrates the wrist encoder, assuming we know the wrist's current angle
   * @param angle current angle that the wrist is physically at, in degrees (0 = horizontal in front of robot, + = up, - = down)
   * @param saveToPrefs true = save the calibration to RobotPreferences
   */
  public void calibrateWristEnc(double angle, boolean saveToPrefs) {
    Robot.robotPrefs.setWristCalibration(getWristEncoderTicksRaw() - degreesToEncoderTicks(angle), saveToPrefs);
  }  
  
	/**
	 * If the angle is reading >/< max/min angle, add/subtract 360 degrees to the wristCalZero accordingly
	 * Note: when the motor is not inverted, upon booting up, an absolute encoder reads a value between 0 and 4096
	 * 		 when the motor is inverted, upon booting up, an absolute encoder reads a value between 0 and -4096
	 * Note: absolute encoder values don't wrap during operation
	 */
	public void adjustWristCalZero() {
    Robot.log.writeLogEcho("Wrist", "Adjust wrist pre", "wrist angle," + getWristAngle() + 
      "raw ticks" + getWristEncoderTicksRaw() + ",wristCalZero," + Robot.robotPrefs.wristCalZero);
		if(getWristAngle() < RobotPreferences.wristMin - 10.0) {
      Robot.log.writeLogEcho("Wrist", "Adjust wrist", "Below min angle");
			Robot.robotPrefs.wristCalZero -= Robot.robotPrefs.encoderTicksPerRevolution;
		}
		else if(getWristAngle() > RobotPreferences.wristMax + 10.0) {
      Robot.log.writeLogEcho("Wrist", "Adjust wrist", "Above max angle");
			Robot.robotPrefs.wristCalZero += Robot.robotPrefs.encoderTicksPerRevolution;
		}
    Robot.log.writeLogEcho("Wrist", "Adjust wrist post", "wrist angle," + getWristAngle() + 
      "raw ticks" + getWristEncoderTicksRaw() + ",wristCalZero," + Robot.robotPrefs.wristCalZero);
	}

  /**
   * Reads whether wrist is at lower limit
   * @return true if wrist is at lower limit, false if not
   */
  public boolean getWristLowerLimit() {
    return wristLimits.isRevLimitSwitchClosed();
  }

  /**
   * Reads whether wrist is at upper limit
   * @return true if wrist is at upper limit, false if not
   */
  public boolean getWristUpperLimit() {
    return wristLimits.isFwdLimitSwitchClosed();
  }

  /**
   * 
   * @return raw encoder ticks (based on encoder zero being at horizontal position)
   */
  public double getWristEncoderTicks() {
    if(Robot.log.getLogLevel() == 1){
      Robot.log.writeLog("Wrist", "Wrist Encoder Ticks", "Wrist Encoder Ticks," + (getWristEncoderTicksRaw() - Robot.robotPrefs.wristCalZero));
    }
    return getWristEncoderTicksRaw() - Robot.robotPrefs.wristCalZero;
  }

  /**
   * 
   * @return raw encoder ticks, adjusted direction (positive is towards stowed, negative is towards lower hard stop)
   */
  public double getWristEncoderTicksRaw() {
    // TODO verify direction, add minus sign if needed
    return wristMotor.getSelectedSensorPosition(0);
  }

  /**
   * 
   * @param encoderTicks encoder ticks
   * @return parameter encoder ticks converted to equivalent degrees
   */
  public double encoderTicksToDegrees(double encoderTicks) {
    return encoderTicks * encoderDegreesPerTicks;
  }

  /**
   * 
   * @param degrees angle in degrees
   * @return parameter degrees converted to equivalent encoder ticks
   */
  public double degreesToEncoderTicks(double degrees) {
    return degrees * encoderTicksPerDegrees;
  }

  /**
   * For use in the wrist subsystem only.  Use getWristAngle() when calling from outside this class.
   * @return current encoder ticks (based on zero) converted to degrees
   */
  private double getWristEncoderDegrees() {
    if(Robot.log.getLogLevel() == 1){
      Robot.log.writeLog("Wrist", "Wrist Encoder Degrees", "Wrist Encoder Degrees," + encoderTicksToDegrees(getWristEncoderTicks()));
    }
    return encoderTicksToDegrees(getWristEncoderTicks());
  }

  /**
   * 
   * @return current encoder ticks converted to degrees
   */
  public double getWristEncoderDegreesRaw() {
    return encoderTicksToDegrees(getWristEncoderTicksRaw());
  }

  /**
	 * Returns the angle that wrist is currently positioned at in degrees.
	 * If the wrist is not calibrated, then returns wristMax.
	 * @return current degree of wrist angle
	 */
  public double getWristAngle() {
    if (Robot.robotPrefs.wristCalibrated) {
      double wristAngle = getWristEncoderDegrees();
      wristAngle = wristAngle % 360; // If encoder wraps around 360 degrees
      wristAngle = (wristAngle > 180) ? wristAngle - 360 : wristAngle; // Change range to -180 to +180
      if (Robot.log.getLogLevel() == 1){
        Robot.log.writeLog("Wrist", "Get Wrist Angle", "Wrist Angle," + wristAngle);
      }
      return wristAngle;
    } else {
      return RobotPreferences.wristMax;
    }
  }

  /**
	 * Returns the angle that wrist is trying to move to in degrees.
	 * If the wrist is not calibrated, then returns wristMax.
	 * @return desired degree of wrist angle
	 */
  public double getCurrentWristTarget() {
    if (Robot.robotPrefs.wristCalibrated) {
      double currentTarget = encoderTicksToDegrees(wristMotor.getClosedLoopTarget(0) - Robot.robotPrefs.wristCalZero);
      if(Robot.log.getLogLevel() == 1){
        Robot.log.writeLog("Wrist", "Wrist Target", "Wrist Target," + currentTarget);
      }
      return currentTarget;
    } else {
      return RobotPreferences.wristMax;
    }
  }

  /**
	 * returns whether encoder is calibrated or not
	 * @return true if encoder is calibrated and working, false if encoder broke
	 */
	public boolean isEncoderCalibrated() {
		return Robot.robotPrefs.wristCalibrated;
  }

  /**
   * Writes information about the subsystem to the filelog
   */
  public void updateWristLog() {
    Robot.log.writeLog("Wrist", "Update Variables",
        "Volts," + wristMotor.getMotorOutputVoltage() + ",Amps," + Robot.pdp.getCurrent(RobotMap.wristMotorPDP) +
        ",WristCalZero," + Robot.robotPrefs.wristCalZero + 
        "Enc Raw," + getWristEncoderTicksRaw() + ",Wrist Angle," + getWristAngle() +
        ",Upper Limit," + getWristUpperLimit() + ",Lower Limit," + getWristLowerLimit()
        );
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }


  
  @Override
  public void periodic() {

    if (Robot.log.getLogRotation() == FileLog.WRIST_CYCLE) {
      SmartDashboard.putBoolean("Wrist calibrated", Robot.robotPrefs.wristCalibrated);
      SmartDashboard.putNumber("Wrist Angle", getWristAngle());
      SmartDashboard.putNumber("Wrist enc raw", getWristEncoderTicksRaw());
			SmartDashboard.putBoolean("Wrist Lower Limit", getWristLowerLimit());
			SmartDashboard.putBoolean("Wrist Upper Limit", getWristUpperLimit());
    }
    
    // Checks if the wrist is not calibrated and automatically calibrates it once the limit switch is pressed
    // If the wrist isn't calibrated at the start of the match, does that mean we can't control the wrist at all?
    if (!Robot.robotPrefs.wristCalibrated) {
      if (getWristUpperLimit()) {
        calibrateWristEnc(RobotPreferences.wristMax, false);
        updateWristLog();
      }
      if (getWristLowerLimit()) {
        calibrateWristEnc(RobotPreferences.wristMin, false);
        updateWristLog();
      }
    }
    
    // Un-calibrates the wrist if the angle is outside of bounds... can we figure out a way to not put this in periodic()?
    if (getWristAngle() > RobotPreferences.wristMax + 5.0 || getWristAngle() < RobotPreferences.wristMin - 5.0) {
      Robot.robotPrefs.setWristUncalibrated();
      updateWristLog();
    }

    if (DriverStation.getInstance().isEnabled()) {
 
      if (Robot.log.getLogRotation() == FileLog.WRIST_CYCLE) {
        updateWristLog();
      }

      /* All of the code below should be gotten rid of for the same reason as the elevator stuff. It doesn't speed anything up in competition - 
      the codriver still has to recognize that the encoders are broken and the wrist is stalled. This is just more code to run in periodic() */
      
      // TODO: Delete everything below this


      // Following code checks whether the encoder is incrementing in the same direction as the 
      // motor is moving and changes control modes based on state of encoder
      
			currEnc = getWristEncoderTicks();
      if (wristMotor.getMotorOutputVoltage() > 5) {
        if (posMoveCount == 0) {
          encSnapShot = getWristEncoderTicks();
        }
        negMoveCount = 0;
        posMoveCount++;
        if (posMoveCount > 3) {
          if ((currEnc - encSnapShot) < 100) {
            setDefaultCommand(new WristWithXBox());
            Robot.robotPrefs.setWristUncalibrated();
            updateWristLog();
          }
          posMoveCount = 0;
        }
      }
      if (wristMotor.getMotorOutputVoltage() < -5) {
        if (negMoveCount == 0) {
          encSnapShot = getWristEncoderTicks();
        }
        posMoveCount = 0;
        negMoveCount++;
        if (negMoveCount > 3) {
          if ((currEnc - encSnapShot) > -100) {
            // If something is broken, it's just as easy for the codriver to press the 
            // joystick button in before moving it. There's no time savings by having
            // this in periodic.
            setDefaultCommand(new WristWithXBox()); 
            Robot.robotPrefs.setWristUncalibrated();
            updateWristLog();
          }
          negMoveCount = 0;
        }
      }
    }
  }
 
}
