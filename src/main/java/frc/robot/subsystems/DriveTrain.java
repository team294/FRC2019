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
import com.kauailabs.navx.frc.AHRS;



import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.DriveWithJoysticks;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.I2C;

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

  // Gyro variables
  private AHRS ahrs;
  private double yawZero = 0;
  
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

    // Configure navX
		try {
			/* Communicate w/navX MXP via the MXP SPI Bus.
			 * Alternatively: I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB
			 * See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for
			 * details.
			 */

			ahrs = new AHRS(I2C.Port.kMXP);

		} catch (RuntimeException ex) {
			DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
		}
		ahrs.zeroYaw();
		// zeroGyroRotation();
	}

  public void tankDrive (double powerLeft, double powerRight) {
    this.robotDrive.tankDrive(powerLeft, powerRight);
  }

  /**
	 * Set the percent output of the left motor.
	 * 
	 * @param powerPct Percent of power -1.0 (reverse) to 1.0 (forward)
	 */
	public void setLeftMotors(double powerPct) {
		//TODO check if direction forward/backward is correct
		leftMotor2.set(ControlMode.PercentOutput, -powerPct);
	}

	/**
	 * Set the percent output of the right motor.
	 * 
	 * @param powerPct Percent of power -1.0 (reverse) to 1.0 (forward)
	 */
	public void setRightMotors(double powerPct) {
		//TODO check if direction forward/backward is correct
		rightMotor2.set(ControlMode.PercentOutput, powerPct);
  }

  /**
	 * Stops all motors.
	 */
	public void stopAllMotors() {
		setLeftMotors(0);
		setRightMotors(0);
	}
  
  /**
	 * Turns voltage compensation on or off for drive motors.
	 * Voltage compensation increases accuracy for autonomous code,
	 * but it decreases maximum velocity/power when driving by joystick.
	 * @param turnOn true=turn on, false= turn off
	 */
	public void setVoltageCompensation(boolean turnOn) {
		leftMotor1.enableVoltageCompensation(turnOn);
		leftMotor2.enableVoltageCompensation(turnOn);
		leftMotor3.enableVoltageCompensation(turnOn);
		rightMotor1.enableVoltageCompensation(turnOn);
		rightMotor2.enableVoltageCompensation(turnOn);
		rightMotor3.enableVoltageCompensation(turnOn);
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
    return leftMotor2.getSelectedSensorPosition(0) - leftEncoderZero;
	}

	/**
	 * Get the position of the right encoder, in encoder ticks since last zeroRightEncoder()
	 * 
	 * @return encoder position, in ticks
	 */
	public double getRightEncoderTicks() {
		return -(rightMotor2.getSelectedSensorPosition(0) - rightEncoderZero);
	}

  public double encoderTicksToInches(double encoderTicks) {
    return (encoderTicks / RobotMap.encoderTicksPerRevolution) * Robot.robotPrefs.wheelCircumference ;
  }
  public double inchesToEncoderTicks(double inches) {
    return (inches / Robot.robotPrefs.wheelCircumference) * RobotMap.encoderTicksPerRevolution;
  }
  public double getLeftEncoderInches() {
    SmartDashboard.putNumber("Left Inches", encoderTicksToInches(getLeftEncoderTicks()));
    return encoderTicksToInches(getLeftEncoderTicks());
  }

  public double getRightEncoderInches() {
    SmartDashboard.putNumber("Right Inches", encoderTicksToInches(getRightEncoderTicks()));
    return encoderTicksToInches(getRightEncoderTicks());
  }

  public double getAverageDistance() {
		return (getRightEncoderInches() + getLeftEncoderInches()) / 2.0;
	}

	/**
	 * Zeros the gyro position in software
	 */
	public void zeroGyroRotation() {
		// set yawZero to gryo angle
		yawZero = ahrs.getAngle();
		// System.err.println("PLZ Never Zero the Gyro Rotation it is not good");
	}

	/**
	 * Resets the gyro position in software to a specified angle
	 * 
	 * @param currentHeading Gyro heading to reset to, in degrees
	 */
	public void setGyroRotation(double currentHeading) {
		// set yawZero to gryo angle, offset to currentHeading
		yawZero = ahrs.getAngle() - currentHeading;
		// System.err.println("PLZ Never Zero the Gyro Rotation it is not good");
	}

	/**
	 * Gets the rotation of the gyro
	 * 
	 * @return Current angle from -180 to 180 degrees
	 */
	public double getGyroRotation() {
		double angle = ahrs.getAngle() - yawZero;
		// Angle will be in terms of raw gyro units (-inf,inf), so you need to convert
		// to (-180, 180]
		angle = angle % 360;
		angle = (angle <= -180) ? (angle + 360) : angle;
    angle = (angle > 180) ? (angle - 360) : angle;
    SmartDashboard.putNumber("Gyro Angle", angle);
		return angle;
  }
  public void setDriveMode(boolean setCoast){
   if(setCoast){
    leftMotor1.setNeutralMode(NeutralMode.Coast);
    leftMotor2.setNeutralMode(NeutralMode.Coast);
    leftMotor3.setNeutralMode(NeutralMode.Coast);
    rightMotor1.setNeutralMode(NeutralMode.Coast);
    rightMotor2.setNeutralMode(NeutralMode.Coast);
    rightMotor3.setNeutralMode(NeutralMode.Coast);

   }else{
    leftMotor1.setNeutralMode(NeutralMode.Coast);
    leftMotor2.setNeutralMode(NeutralMode.Brake);
    leftMotor3.setNeutralMode(NeutralMode.Brake);
    rightMotor1.setNeutralMode(NeutralMode.Brake);
    rightMotor2.setNeutralMode(NeutralMode.Brake);
    rightMotor3.setNeutralMode(NeutralMode.Brake);

   }
   
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
    double startSpeed = 0.5;  // + (1.0/800.0 * (Robot.vision.distanceFromTarget() - 50));
    double lJoystickPercent = Robot.oi.leftJoystick.getY();
    double lPercentOutput = startSpeed + (gainConstant * xVal);
    double rPercentOutput = startSpeed - (gainConstant * xVal);
    System.out.println("lPercentOut, rPercentOut "+lPercentOutput+" "+rPercentOutput);
    // SEE ROB ON THIS about area == 0
/*
    if (Robot.vision.distanceFromTarget() > 30 && Robot.vision.areaFromCamera != 0 && lJoystickPercent == 0) {
        this.robotDrive.tankDrive(lPercentOutput, rPercentOutput);
    } else if (Robot.vision.distanceFromTarget() > 30 && Robot.vision.areaFromCamera != 0) {
      this.robotDrive.tankDrive(lPercentOutput - lJoystickPercent, rPercentOutput - lJoystickPercent);
    } else {
      this.robotDrive.tankDrive(0, 0);
    }
    Robot.log.writeLog("DriveTrain", "Vision Driving", "Degrees from Target," + xVal + ",Joystick Ouput," + lJoystickPercent + ",Inches from Target," + Robot.vision.distanceFromTarget()
    + ",Target Area," + Robot.vision.areaFromCamera);
*/
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
   Robot.log.writeLog("DriveTrain", "Vision Turning", "Degrees from Target," + xVal + ",Target Area," + Robot.vision.areaFromCamera);
  }

  public void turnToLine() {
    double lpercentPower = 0;
    double rpercentPower = 0;
    if (Robot.lineFollowing.isLinePresent(1)) {
    }
    if (Robot.lineFollowing.isLinePresent(0)) {
      //turn left
    }
    if (Robot.lineFollowing.isLinePresent(2)) {
      //turn right
    }
  }

  public void turnToAngle() {

   
    double turnAngle = 0;
    double currentAngleFake = 0;
    double fixSpeed = 0.4;
    double percentOutput;
    double error = 1;
    //double rPercentOutput;   
    double currentAngle = Robot.driveTrain.getGyroRotation();
    double targetAngle = 90; // Depends on a button press TBD
    boolean sameSide = false;
    boolean inRightRange = false;

    
  //  turnAngle = (180/Math.PI)*(Math.acos(Math.cos(currentAngle)*Math.cos(targetAngle)+Math.sin(currentAngle)*Math.sin(targetAngle)));
  

    if (((targetAngle > 0) && (currentAngle > 0)|| ((targetAngle < 0) && (currentAngle < 0)))){
      sameSide = true;
      turnAngle = (Math.abs(currentAngle) - Math.abs(targetAngle));
    } else {
      sameSide = false;
      if(180 > (180 - Math.abs(currentAngle) + (180 - Math.abs(targetAngle)))){
        turnAngle = (180 - Math.abs(currentAngle) + 180 - Math.abs(targetAngle));
      } else {
        turnAngle = Math.abs(currentAngle) + Math.abs(targetAngle);
      }
      
    }


    if(((targetAngle < currentAngle) && (targetAngle > -1*(180-currentAngle)) && (currentAngle>0))) {
      inRightRange = true;
    } else if(((targetAngle > currentAngle) && (targetAngle < (180+currentAngle)) && (currentAngle>0))) {
      inRightRange = true; // IN RIGHT RANGE NEEDS TO BER DOUBLE CHECKED THIS LINE LOGIC CHECK ALMOST DONE
    }


     //turnAngle *= (-1);

 System.out.println("turn angle is " + turnAngle + "current angle is " + currentAngle);
 System.out.println("is right boolean " + inRightRange);
   
      percentOutput = fixSpeed + (.05*turnAngle);
      

      if (inRightRange) {
        this.robotDrive.tankDrive(percentOutput, -percentOutput);
      } else if (!inRightRange) {
        this.robotDrive.tankDrive(-percentOutput, percentOutput);
      } else {
        this.robotDrive.tankDrive(0, 0);
      }

    } 
  }


