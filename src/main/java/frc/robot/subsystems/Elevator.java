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

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.SensorCollection;
/** 
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.ControlType;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
**/
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
  private SensorCollection elevatorLowerLimit;

  private int periodicCount = 0;
  private int idleCount= 0;
  private double prevEnc = 0.0;
  private double currEnc = 0.0;

  public double rampRate = .005; 
  public double kP = 1;
  public double kI = 0;
  public double kD = 0;
  public double kFF = 0;
  public int kIz = 0;
  public double kMaxOutput = 1.0;		//  up max output
  public double kMinOutput = -0.5;		//  down max output

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
	elevatorMotor1.setInverted(true);
    elevatorMotor1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
	elevatorMotor1.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
	elevatorMotor1.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
	
	elevatorLowerLimit = elevatorMotor1.getSensorCollection();
	zeroElevatorEnc();

	elevatorMotor1.config_kP(0, kP);
	elevatorMotor1.config_kI(0, kI);
	elevatorMotor1.config_kD(0, kD);
	elevatorMotor1.config_kF(0, kFF);
	elevatorMotor1.config_IntegralZone(0, kIz);
	elevatorMotor1.configClosedloopRamp(rampRate);
	elevatorMotor1.configPeakOutputForward(kMaxOutput);
	elevatorMotor1.configPeakOutputReverse(kMinOutput);

	elevatorMotor1.clearStickyFaults();
	elevatorMotor2.clearStickyFaults();
	elevatorMotor1.setNeutralMode(NeutralMode.Brake);
	elevatorMotor2.setNeutralMode(NeutralMode.Brake);
	}
	
	/* 
	powerLeft and powerRight are between -1.0 and 1.0
	*/
	public void setElevatorMotorPercentOutput(double percentOutput) {
		elevatorMotor1.set(percentOutput);
	}

	public void setElevatorPos(double inches) {
		elevatorMotor1.set(ControlMode.Position, inchesToEncoderTicks(inches));
	}

	public void stopElevator() {
		setElevatorMotorPercentOutput(0.0);
	}

	public void zeroElevatorEnc() {
		elevatorMotor1.setSelectedSensorPosition(0, 0, 0);
	}

	public double getElevatorEncTicks() {
		return elevatorMotor1.getSelectedSensorPosition(0);
	}

	public double encoderTicksToInches(double encoderTicks) {
		return (encoderTicks / RobotMap.encoderTicksPerRevolution);// * Robot.robotPrefs.elevatorGearCircumference;
	}
	/**
	 * to make easier for testing
	 * @param encoderTicks
	 * @return
	 */
	public double encoderTicksToRevolutions(double encoderTicks) {
		return encoderTicks / RobotMap.encoderTicksPerRevolution;
	}

	public double inchesToEncoderTicks(double inches) {
		return (inches / Robot.robotPrefs.elevatorGearCircumference) * RobotMap.encoderTicksPerRevolution;
	}

	public double getElevatorEncInches() {
		return encoderTicksToInches(getElevatorEncTicks());
	}

	public boolean getElevatorLowerLimit() {
		return elevatorLowerLimit.isFwdLimitSwitchClosed();
	}

	public void updateElevatorLog() {
		Robot.log.writeLog("Elevator", "Update Variables",
		"Elev1 Volts," + elevatorMotor1.getBusVoltage() + ",Elev2 Volts," + elevatorMotor2.getBusVoltage() +
		",Elev1 Amps," + Robot.pdp.getCurrent(RobotMap.elevatorMotor1PDP) + ",Elev2 Amps," + Robot.pdp.getCurrent(RobotMap.elevatorMotor2PDP) +
		",Elev Enc Ticks," + getElevatorEncTicks() + ",Elev Enc Inches," + getElevatorEncInches());
	}
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
	setDefaultCommand(new StopElevator());
  }

  @Override
  public void periodic() {
	SmartDashboard.putNumber("Enc Rev", encoderTicksToRevolutions(getElevatorEncTicks()));
	SmartDashboard.putNumber("Enc Inch", getElevatorEncInches());
	SmartDashboard.putNumber("Enc Tick", getElevatorEncTicks());
	SmartDashboard.putBoolean("Forward", getElevatorLowerLimit());
    if (DriverStation.getInstance().isEnabled()) {
		prevEnc = currEnc;
		currEnc = getElevatorEncTicks();
		if (currEnc == prevEnc) {
			idleCount++;
		}
		else {
			idleCount = 0;
		}
// TODO Audrey - explain this to Rob?
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
