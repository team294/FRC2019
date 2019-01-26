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
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.StopElevator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.ControlType;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;

/**
 * Add your docs here.
 */
public class Elevator extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.  
  //private CANSparkMax elevatorMotor1;
  //private CANSparkMax elevatorMotor2;
  //private CANEncoder elevatorEnc;
  //private CANPIDController elevatorPID;
  //private CANDigitalInput elevatorLowerLimit;
  //TODO comment out if they miraculously switch back to Sparks

  private WPI_TalonSRX elevatorMotor1;
  private BaseMotorController elevatorMotor2;
  private DigitalInput elevatorLowerLimit = new DigitalInput(RobotMap.elevatorLowerLimit);

  private int periodicCount = 0;
  private int idleCount= 0;
  private double encoderZero = 0;
  private double prevEnc = 0.0;
  private double currEnc = 0.0;

  /* public double rampRate = .005; 
  public double kP = 1;
  public double kI = 0;
  public double kD = 0;
  public double kIz = 0;
  public double kFF = 0;
  public double kMaxOutput = 1;
  public double kMinOutput = -1; */

  public Elevator() {
	/* elevatorMotor1 = new CANSparkMax(RobotMap.elevatorMotor1, MotorType.kBrushless);
	elevatorMotor2 = new CANSparkMax(RobotMap.elevatorMotor2, MotorType.kBrushless);
	elevatorEnc = elevatorMotor1.getEncoder();
	elevatorPID = elevatorMotor1.getPIDController();
	elevatorLowerLimit = elevatorMotor1.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
	elevatorMotor2.follow(elevatorMotor1, true); 
	elevatorMotor1.clearFaults();	
	elevatorMotor2.clearFaults();
	elevatorLowerLimit.enableLimitSwitch(true);
	elevatorPID.setP(kP);
	elevatorPID.setI(kI);
	elevatorPID.setD(kD);
	elevatorPID.setIZone(kIz);
	elevatorPID.setFF(kFF);
	elevatorMotor1.setRampRate(rampRate);
	elevatorPID.setOutputRange(kMinOutput, kMaxOutput); 
	*/

	elevatorMotor1 = new WPI_TalonSRX(RobotMap.elevatorMotor1);
	elevatorMotor2 = new WPI_VictorSPX(RobotMap.elevatorMotor2);
	elevatorMotor2.follow(elevatorMotor1);
	elevatorMotor2.setInverted(true);
    elevatorMotor1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
	zeroElevatorEnc();

	elevatorMotor1.clearStickyFaults();
	elevatorMotor2.clearStickyFaults();
	elevatorMotor1.setNeutralMode(NeutralMode.Brake);
	elevatorMotor2.setNeutralMode(NeutralMode.Brake);
	}
	
	/* 
	powerLeft and powerRight are between -1.0 and 1.0
	*/
	public void setElevatorMotorPercentPower(double percentPower) {
		elevatorMotor1.set(percentPower);
	}

	/* public void setElevatorPos(double inches) {
		elevatorPID.setReference(inchesToEncoderRevolutions(inches) + encoderZero, ControlType.kPosition);
	} TODO uncomment when PID for this is figured out */

	public void stopElevator() {
		setElevatorMotorPercentPower(0.0);
	}

	public void zeroElevatorEnc() {
		encoderZero = elevatorMotor1.getSelectedSensorPosition(0);
	}

	public double getElevatorEncRevolutions() {
		return elevatorMotor1.getSelectedSensorPosition(0) - encoderZero;
	}

	public double encoderRevolutionsToInches(double encoderRevs) {
		return encoderRevs * Robot.robotPrefs.elevatorGearCircumference;
	}

	public double inchesToEncoderRevolutions(double inches) {
		return inches / Robot.robotPrefs.elevatorGearCircumference;
	}

	public double getElevatorEncInches() {
		return encoderRevolutionsToInches(getElevatorEncRevolutions());
	}

	public boolean getElevatorLowerLimit() {
		return elevatorLowerLimit.get();
	}

	public void updateElevatorLog() {
		Robot.log.writeLog("Elevator", "Update Variables",
		"Elev1 Volts," + elevatorMotor1.getBusVoltage() + ",Elev2 Volts," + elevatorMotor2.getBusVoltage() +
		",Elev1 Amps," + Robot.pdp.getCurrent(RobotMap.elevatorMotor1PDP) + ",Elev2 Amps," + Robot.pdp.getCurrent(RobotMap.elevatorMotor2PDP) +
		",Elev Enc Revs," + getElevatorEncRevolutions() + ",Elev Enc Inches," + getElevatorEncInches());
	}
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
	setDefaultCommand(new StopElevator());
  }

  @Override
  public void periodic() {
	SmartDashboard.putNumber("Enc Zero", encoderZero);
	SmartDashboard.putNumber("Enc Val", getElevatorEncRevolutions());
	SmartDashboard.putBoolean("Forward", getElevatorLowerLimit());
    if (DriverStation.getInstance().isEnabled()) {
		prevEnc = currEnc;
		currEnc = getElevatorEncRevolutions();
		if (currEnc == prevEnc) {
			idleCount++;
		}
		else {
			idleCount = 0;
		}

		if(idleCount>=50) {
			if((++periodicCount) >= 25) {
				updateElevatorLog();
				periodicCount = 0;
			}
		}
		else {
			updateElevatorLog();
		}
    }
  }
}
