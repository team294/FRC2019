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
import frc.robot.commands.StopElevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;

/**
 * Add your docs here.
 */
public class Elevator extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.  

  private CANSparkMax elevatorMotor1;
  private CANSparkMax elevatorMotor2;
  private CANEncoder elevatorEnc;
  private CANPIDController elevatorPID;

  private int periodicCount = 0;

  private double encoderZero = 0;

  public double rampRate = .005; 
  public double kP = 1;
  public double kI = 0;
  public double kD = 0;
  public double kIz = 0;
  public double kFF = 0;
  public double kMaxOutput = 1;
  public double kMinOutput = -1;

  public Elevator() {
	elevatorMotor1 = new CANSparkMax(RobotMap.elevatorMotor1, MotorType.kBrushless);
	elevatorMotor2 = new CANSparkMax(RobotMap.elevatorMotor2, MotorType.kBrushless);
	elevatorEnc = elevatorMotor1.getEncoder();
	elevatorPID = elevatorMotor1.getPIDController();
	elevatorMotor2.follow(elevatorMotor1);
	elevatorMotor1.clearFaults();	
	elevatorMotor2.clearFaults();
	zeroElevatorEnc();
	elevatorPID.setP(kP);
	elevatorPID.setI(kI);
	elevatorPID.setD(kD);
	elevatorPID.setIZone(kIz);
	elevatorPID.setFF(kFF);
	elevatorMotor1.setRampRate(rampRate);
	elevatorPID.setOutputRange(kMinOutput, kMaxOutput);
	}
	
	/* 
	powerLeft and powerRight are between -1.0 and 1.0
	*/
	public void setElevatorMotorPercentPower(double percentPower) {
		elevatorMotor1.set(percentPower);
	}

	public void setElevatorPos(double inches) {
		elevatorPID.setReference(inchesToEncoderRevolutions(inches) + encoderZero, ControlType.kPosition);
	}

	public void stopElevator() {
		setElevatorMotorPercentPower(0.0);
	}

	public void zeroElevatorEnc() {
		encoderZero = elevatorEnc.getPosition();
	}

	public double getElevatorEncRevolutions() {
		return elevatorEnc.getPosition() - encoderZero;
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

	public void updateElevatorLog() {
		Robot.log.writeLog("Elevator", "Update Variables",
		"Elev1 Volts," + elevatorMotor1.getBusVoltage() + ",Elev2 Volts," + elevatorMotor2.getBusVoltage() +
		",Elev1 Amps," + elevatorMotor1.getOutputCurrent() + ",Elev2 Amps," + elevatorMotor2.getOutputCurrent() +
		",Elev1 Temp," + elevatorMotor1.getMotorTemperature() + ",Elev2 Temp," + elevatorMotor2.getMotorTemperature() +
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
    if (DriverStation.getInstance().isEnabled()) {
      if ((++periodicCount) >= 25) {
        updateElevatorLog();
        periodicCount=0;  
      }
    }
  }
}
