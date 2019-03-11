/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.I2C;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * Add your docs here.
 */
public class NeoDriveTrain extends DriveTrain {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private CANSparkMax leftMotor1 = new CANSparkMax(RobotMap.leftMotor1, MotorType.kBrushless);
  private CANSparkMax leftMotor2 = new CANSparkMax(RobotMap.leftMotor2, MotorType.kBrushless);
  private CANSparkMax leftMotor3 = new CANSparkMax(RobotMap.leftMotor3, MotorType.kBrushless);
  private CANSparkMax rightMotor1 = new CANSparkMax(RobotMap.rightMotor1, MotorType.kBrushless);
  private CANSparkMax rightMotor2 = new CANSparkMax(RobotMap.rightMotor2, MotorType.kBrushless);
  private CANSparkMax rightMotor3 = new CANSparkMax(RobotMap.rightMotor3, MotorType.kBrushless);
  
  private CANEncoder leftDriveEnc;
  private CANEncoder rightDriveEnc;

  private double leftEncoderZero = 0, rightEncoderZero = 0;

  private AHRS ahrs;
  private double yawZero = 0;

  private SensorCollection lShaftEncoder = new WPI_TalonSRX(10).getSensorCollection(); // THESE ARE MAGIC NUMBERS
  private SensorCollection rShaftEncoder = new WPI_TalonSRX(20).getSensorCollection(); // REPLACE WITH THE TALON IDs ACTUALLY PLUGGED INTO

  public final DifferentialDrive robotDrive = new DifferentialDrive(leftMotor2, rightMotor2);

  public NeoDriveTrain() {

    leftMotor1.setIdleMode(IdleMode.kBrake);
    leftMotor2.setIdleMode(IdleMode.kBrake);
    leftMotor3.setIdleMode(IdleMode.kBrake);
    rightMotor1.setIdleMode(IdleMode.kBrake);
    rightMotor2.setIdleMode(IdleMode.kBrake);
    rightMotor3.setIdleMode(IdleMode.kBrake);

        
    leftDriveEnc = leftMotor2.getEncoder();
    rightDriveEnc = rightMotor2.getEncoder();

    // Need to set inverted?

    zeroLeftEncoder();
    zeroRightEncoder();
    
    leftMotor1.follow(leftMotor2);
    leftMotor3.follow(leftMotor2);
    rightMotor1.follow(rightMotor2);
    rightMotor3.follow(rightMotor2);

    leftMotor1.clearFaults();
    leftMotor2.clearFaults();
    leftMotor3.clearFaults();
    rightMotor1.clearFaults();
    rightMotor2.clearFaults();
    rightMotor3.clearFaults();

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

  @Override
	public void driveAtCurve(double speedPct, double curve) {
		robotDrive.curvatureDrive(speedPct, curve, false);
  }

  @Override
  public void tankDrive(double powerLeft, double powerRight) {
    robotDrive.tankDrive(powerLeft, powerRight);
  }

  @Override
  public void setLeftMotors(double percent) {
    leftMotor2.set(-percent);
  }

  @Override
  public void setRightMotors(double percent) {
    rightMotor2.set(percent);
  }

  @Override
  public double getGyroRaw() {
    return ahrs.getAngle();
  }

  @Override
	public void setVoltageCompensation(boolean turnOn) {
		if (turnOn) {
      leftMotor1.enableVoltageCompensation(12.0);
		  leftMotor2.enableVoltageCompensation(12.0);
		  leftMotor3.enableVoltageCompensation(12.0);
		  rightMotor1.enableVoltageCompensation(12.0);
		  rightMotor2.enableVoltageCompensation(12.0);
      rightMotor3.enableVoltageCompensation(12.0);
    } else {
      leftMotor1.disableVoltageCompensation();
      leftMotor2.disableVoltageCompensation();
      leftMotor3.disableVoltageCompensation();
      rightMotor1.disableVoltageCompensation();
      rightMotor2.disableVoltageCompensation();
      rightMotor3.disableVoltageCompensation();
    }
  }
  
  @Override
	public void zeroLeftEncoder() {
		leftEncoderZero = leftDriveEnc.getPosition();
	}

	@Override
	public void zeroRightEncoder() {
    rightEncoderZero = rightDriveEnc.getPosition();
  }
  
  @Override
	public double getLeftEncoderTicks() {
    return leftDriveEnc.getPosition() - leftEncoderZero;
	}

	@Override
	public double getRightEncoderTicks() {
		return -(rightDriveEnc.getPosition() - rightEncoderZero);
  }

  // TODO: add methods for using the shaft encoders
  
  @Override
  public double encoderTicksToInches(double rotations) {
    return rotations * Robot.robotPrefs.wheelCircumference; // This is not right
  }

  @Override
  public void setDriveMode(boolean setCoast) {
    if (setCoast) {
     leftMotor1.setIdleMode(IdleMode.kCoast);
     leftMotor2.setIdleMode(IdleMode.kCoast);
     leftMotor3.setIdleMode(IdleMode.kCoast);
     rightMotor1.setIdleMode(IdleMode.kCoast);
     rightMotor2.setIdleMode(IdleMode.kCoast);
     rightMotor3.setIdleMode(IdleMode.kCoast);
    } else {
     leftMotor1.setIdleMode(IdleMode.kBrake);
     leftMotor2.setIdleMode(IdleMode.kBrake);
     leftMotor3.setIdleMode(IdleMode.kBrake);
     rightMotor1.setIdleMode(IdleMode.kBrake);
     rightMotor2.setIdleMode(IdleMode.kBrake);
     rightMotor3.setIdleMode(IdleMode.kBrake);
    }
    
  }

  @Override
  public void updateDriveLog (boolean logWhenDisabled) {
    Robot.log.writeLog(logWhenDisabled, "DriveTrain", "Update Variables",
      "Drive L1 Volts," + leftMotor1.getBusVoltage() + ",Drive L2 Volts," + leftMotor2.getBusVoltage() + ",Drive L3 Volts," + leftMotor3.getBusVoltage() +
      ",Drive L1 Amps," + Robot.pdp.getCurrent(RobotMap.leftMotor1PDP) + ",Drive L2 Amps," + Robot.pdp.getCurrent(RobotMap.leftMotor2PDP) + ",Drive L3 Amps," + Robot.pdp.getCurrent(RobotMap.leftMotor3PDP) + 
      ",Drive R1 Volts," + rightMotor1.getBusVoltage() + ",Drive R2 Volts," + rightMotor2.getBusVoltage() + ",Drive R3 Volts," + rightMotor3.getBusVoltage() + 
      ",Drive R1 Amps," + Robot.pdp.getCurrent(RobotMap.rightMotor1PDP) + ",Drive R2 Amps," + Robot.pdp.getCurrent(RobotMap.rightMotor2PDP) + ",Drive R3 Amps," + Robot.pdp.getCurrent(RobotMap.rightMotor3PDP) + 
      ",L Enc Ticks," + getLeftEncoderTicks() + ",L Drive Inches," + getLeftEncoderInches() + 
      ",R Enc Ticks," + getRightEncoderTicks() + ",R Drive Inches," + getRightEncoderInches() + 
      ",High Gear," + Robot.shifter.isShifterInHighGear());
  }

}
