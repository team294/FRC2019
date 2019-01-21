/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


package frc.robot.subsystems;   // had to change from just frc.robot.subsystems ?


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.DriveWithJoysticks;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Drive Train subsystem.  
 */
public class DriveTrain extends Subsystem {
  // This is to test with the 2018 drive base.  The 2019 drive base will use 4 Victor SPX controllers for follower motors 1 and 3
  //private final WPI_TalonSRX leftMotor1 = new WPI_TalonSRX(RobotMap.leftMotor1);
  //private final WPI_TalonSRX leftMotor3 = new WPI_TalonSRX(RobotMap.leftMotor3);
  //private final WPI_TalonSRX rightMotor1 = new WPI_TalonSRX(RobotMap.rightMotor1);
  //private final WPI_TalonSRX rightMotor3 = new WPI_TalonSRX(RobotMap.rightMotor3);
  private final BaseMotorController leftMotor1;
  private final WPI_TalonSRX leftMotor2 = new WPI_TalonSRX(RobotMap.leftMotor2);
  private final BaseMotorController leftMotor3;
  private final BaseMotorController rightMotor1;
  private final WPI_TalonSRX rightMotor2 = new WPI_TalonSRX(RobotMap.rightMotor2);
  private final BaseMotorController rightMotor3;
  public final DifferentialDrive robotDrive = new DifferentialDrive(leftMotor2, rightMotor2);

  private int periodicCount = 0;
  
  private double leftEncoderZero = 0, rightEncoderZero = 0;

  public DriveTrain() {

    // Reads the robot prefs to check if using Victors or Talons for alternate motor controllers
    if (Robot.robotPrefs.prototypeRobot) { // true means Talons
      leftMotor1 = new WPI_TalonSRX(RobotMap.leftMotor1);
      leftMotor3 = new WPI_TalonSRX(RobotMap.leftMotor3);
      rightMotor1 = new WPI_TalonSRX(RobotMap.rightMotor1);
      rightMotor3 = new WPI_TalonSRX(RobotMap.rightMotor3);

      leftMotor1.set(ControlMode.Follower, RobotMap.leftMotor2);
      leftMotor3.set(ControlMode.Follower, RobotMap.leftMotor2);
      rightMotor1.set(ControlMode.Follower, RobotMap.rightMotor2);
      rightMotor3.set(ControlMode.Follower, RobotMap.rightMotor2);
    }
    else {
      leftMotor1 = new WPI_VictorSPX(RobotMap.leftMotor1);
      leftMotor3 = new WPI_VictorSPX(RobotMap.leftMotor3);
      rightMotor1 = new WPI_VictorSPX(RobotMap.rightMotor1);
      rightMotor3 = new WPI_VictorSPX(RobotMap.rightMotor3);

      leftMotor1.follow(leftMotor2);
      leftMotor3.follow(leftMotor2);
      rightMotor1.follow(rightMotor2);
      rightMotor3.follow(rightMotor2);
    }
    leftMotor2.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		rightMotor2.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		zeroLeftEncoder();
		zeroRightEncoder();

    leftMotor1.setInverted(true);
    leftMotor2.setInverted(true);
    leftMotor3.setInverted(true);
    rightMotor1.setInverted(true);
    rightMotor2.setInverted(true);
    rightMotor3.setInverted(true);

    leftMotor1.clearStickyFaults(0);
    leftMotor2.clearStickyFaults(0);
    leftMotor3.clearStickyFaults(0);
    rightMotor1.clearStickyFaults(0);
    rightMotor2.clearStickyFaults(0);
    rightMotor3.clearStickyFaults(0);

    leftMotor1.setNeutralMode(NeutralMode.Brake);
    leftMotor2.setNeutralMode(NeutralMode.Brake);
    leftMotor3.setNeutralMode(NeutralMode.Brake);
    rightMotor1.setNeutralMode(NeutralMode.Brake);
    rightMotor2.setNeutralMode(NeutralMode.Brake);
    rightMotor3.setNeutralMode(NeutralMode.Brake);
  }

  public void tankDrive (double powerLeft, double powerRight) {
    this.robotDrive.tankDrive(powerLeft, powerRight);
  }
  
  /**
	 * Zeros the left encoder position in software
	 */
	public void zeroLeftEncoder() {
		leftEncoderZero = leftMotor2.getSelectedSensorPosition(0);
	}

	/**
	 * Zeros the right encoder position in software
	 */
	public void zeroRightEncoder() {
    rightEncoderZero = rightMotor2.getSelectedSensorPosition(0);
	}

	/**
	 * Get the position of the left encoder, in encoder ticks since last zeroLeftEncoder()
	 * 
	 * @return encoder position, in ticks
	 */
	public double getLeftEncoderTicks() {
    return leftMotor2.getSelectedSensorPosition(0) + leftEncoderZero;
	}

	/**
	 * Get the position of the right encoder, in encoder ticks since last zeroRightEncoder()
	 * 
	 * @return encoder position, in ticks
	 */
	public double getRightEncoderTicks() {
		return rightMotor2.getSelectedSensorPosition(0) + rightEncoderZero;
	}

  public double encoderTicksToInches(double encoderTicks) {
    return (encoderTicks / RobotMap.encoderTicksPerRevolution) * Robot.robotPrefs.wheelCircumference * Robot.robotPrefs.driveTrainDistanceFudgeFactor;
  }
  public double inchesToEncoderTicks(double inches) {
    return (inches / Robot.robotPrefs.wheelCircumference / Robot.robotPrefs.driveTrainDistanceFudgeFactor) * RobotMap.encoderTicksPerRevolution;
  }
  public double getLeftEncoderInches() {
    return encoderTicksToInches(getLeftEncoderTicks());
  }

  public double getRightEncoderInches() {
    return encoderTicksToInches(getRightEncoderTicks());
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new DriveWithJoysticks());
  }
  @Override
  public void periodic() {

    if (DriverStation.getInstance().isEnabled()) {
      if ((++periodicCount) >= 25) {
        updateDriveLog();
        periodicCount=0;  
      }
    }
  }
  
  public void updateDriveLog () {
    Robot.log.writeLog("DriveTrain", "Update Variables",
      "Drive L1 Volts," + leftMotor1.getMotorOutputVoltage() + ",Drive L2 Volts," + leftMotor2.getMotorOutputVoltage() + ",Drive L3 Volts," + leftMotor3.getMotorOutputVoltage() +
      ",Drive L1 Amps," + Robot.pdp.getCurrent(RobotMap.leftMotor1PDP) + ",Drive L2 Amps," + Robot.pdp.getCurrent(RobotMap.leftMotor2PDP) + ",Drive L3 Amps," + Robot.pdp.getCurrent(RobotMap.leftMotor3PDP) + 
      ",Drive R1 Volts," + rightMotor1.getMotorOutputVoltage() + ",Drive R2 Volts," + rightMotor2.getMotorOutputVoltage() + ",Drive R3 Volts," + rightMotor3.getMotorOutputVoltage() + 
      ",Drive R1 Amps," + Robot.pdp.getCurrent(RobotMap.rightMotor1PDP) + ",Drive R2 Amps," + Robot.pdp.getCurrent(RobotMap.rightMotor2PDP) + ",Drive R3 Amps," + Robot.pdp.getCurrent(RobotMap.rightMotor3PDP) + 
      ",L Enc Zero," + leftEncoderZero + ",L Enc Ticks," + getLeftEncoderTicks() + ",L Drive Inches," + getLeftEncoderInches() + 
      ",R Enc Zero," + rightEncoderZero + ",R Enc Ticks," + getRightEncoderTicks() + ",R Drive Inches," + getRightEncoderInches() + 
      ",High Gear," + Robot.shifter.isShifterInHighGear());
  }

/**
   * Drives towards target and stops in front of it using speed from left joystick
   * This may change depending on driver preferences
   */
  public void driveToCrosshair() {

    double gainConstant = 1.0/30.0;
    double xVal = Robot.vision.xValue.getDouble(0);
    // 50 inches subtracted from the distance to decrease the speed
    double startSpeed =-0.5;  // + (1.0/800.0 * (Robot.vision.distanceFromTarget() - 50));
    double lJoystickPercent = Robot.oi.leftJoystick.getY();
    double lPercentOutput = startSpeed + (gainConstant * xVal);
    double rPercentOutput = startSpeed - (gainConstant * xVal);
    System.out.println("lPercentOut, rPercentOut "+lPercentOutput+" "+rPercentOutput);
    // SEE ROB ON THIS about area == 0
    if (Robot.vision.distanceFromTarget() > 30 && Robot.vision.areaFromCamera != 0 && lJoystickPercent == 0) {
        this.robotDrive.tankDrive(lPercentOutput, rPercentOutput);
    } else if (Robot.vision.distanceFromTarget() > 30 && Robot.vision.areaFromCamera != 0) {
      this.robotDrive.tankDrive(lPercentOutput - lJoystickPercent, rPercentOutput - lJoystickPercent);
    } else {
      this.robotDrive.tankDrive(0, 0);
    }
    Robot.log.writeLog("DriveTrain", "Vision Driving", "Degrees from Target," + xVal + ",Joystick Ouput," + lJoystickPercent + ",Inches from Target," + Robot.vision.distanceFromTarget()
    + ",Target Area," + Robot.vision.areaFromCamera);
  }

   /**
   * Turns in place to target
   */
  public void turnToCrosshair() {
    double gainConstant = 1.0/30.0;
    double xVal = Robot.vision.xValue.getDouble(0);
    double fixSpeed = 0.4;
    double lPercentOutput = fixSpeed + (gainConstant * xVal);
    double rPercentOutput = fixSpeed - (gainConstant * xVal);
    
    if (xVal > 0.5) {
      this.robotDrive.tankDrive(lPercentOutput, -lPercentOutput);
    } else if (xVal < -0.5) {
      this.robotDrive.tankDrive(-rPercentOutput, rPercentOutput);
    } else {
      this.robotDrive.tankDrive(0, 0);
   }
   Robot.log.writeLog("DriveTrain", "Vision Turning", "Degrees from Target," + xVal + ",Inches from Target," + Robot.vision.distanceFromTarget() + ",Target Area," + Robot.vision.areaFromCamera);
  }

  public void DriveOnLine() {
    double lPercentPower = 0;
    double rPercentPower = 0;
    double baseSpeed = 1;
    //LineFollowing name = new LineFollowing();
    int lineNum = Robot.lineFollowing.lineNumber();
    if (lineNum == 0) {
      // Straight
      lPercentPower = .55*baseSpeed;
      rPercentPower = .55*baseSpeed;
    } else if (lineNum == 1) {
      // Turn left slight?
      lPercentPower = .6*baseSpeed;
      rPercentPower = 0*baseSpeed;
    } else if (lineNum == -1) {
      // Turn right slight?
      lPercentPower = 0*baseSpeed;
      rPercentPower = .6*baseSpeed;
    } else if (lineNum == -2) {
      // Turn left
      lPercentPower = .8*baseSpeed;
      rPercentPower = -.8*baseSpeed;
    } else if (lineNum == 2) {
      // Turn right
      /*while (Robot.lineFollowing.isLinePresent(2) == false){
        this.robotDrive.tankDrive(.5, .5);
      }*/
      lPercentPower = -.8*baseSpeed;
      rPercentPower = .8*baseSpeed;
    } else {
      // Stop
      lPercentPower = 0;
      rPercentPower = 0;
    }
    System.out.println("Base Speed " + baseSpeed +" Left percent power: " + lPercentPower + " Right Percent Power " + rPercentPower);
    this.robotDrive.tankDrive(lPercentPower, rPercentPower);
  }
}


