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
import frc.robot.commands.ElevatorWithXBox;
import frc.robot.utilities.FileLog;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.SensorCollection;

/**
 * Add your docs here.
 */
public class Elevator extends Subsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	private WPI_TalonSRX elevatorMotor1;
	private WPI_TalonSRX elevatorMotor2;
	private SensorCollection elevatorLimits;

	private int posMoveCount = 0; // increments every cycle the elevator moves up
	private int negMoveCount = 0; // increments every cycle the elevator moves down
	private int motorFaultCount = 0; // increments every cycle the motor detects an issue
	private double currEnc = 0.0; // current recorded encoder value
	private double encSnapShot = 0.0; // snapshot of encoder value used to make sure encoder is working
	private boolean elevEncOK = true; // true is encoder working, false is encoder broken
	private boolean elevatorMode; // true is automated, false is manual mode

	private double rampRate = .005;
	private double kP = 0.5;
	private double kI = 0;
	private double kD = 0;
	private double kFF = 0;
	private int kIz = 0;
	private double kMaxOutput = 1.0; // up max output
	private double kMinOutput = -1.0; // down max output

	public Elevator() {
		elevatorMotor1 = new WPI_TalonSRX(RobotMap.elevatorMotor1);
		elevatorMotor2 = new WPI_TalonSRX(RobotMap.elevatorMotor2);
		elevatorMotor2.follow(elevatorMotor1);
		elevatorMotor1.setInverted(false);
		elevatorMotor2.setInverted(false);
		elevatorMotor1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		elevatorMotor1.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
		elevatorMotor1.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

		elevatorLimits = elevatorMotor1.getSensorCollection();
		checkAndZeroElevatorEnc();

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

		if(getElevatorLowerLimit() && getElevatorEncTicks() == 0) {
			elevatorMode = true;
		}
		else {
			elevatorMode = false; // start the elevator in manual mode unless it is properly zeroed
		}
	}

	/**
	 * @param percentOutput between -1.0 (down) and 1.0 (up)
	 */
	public void setElevatorMotorPercentOutput(double percentOutput) {
		elevatorMotor1.set(ControlMode.PercentOutput, percentOutput);
	}

	/**
	 * only works when encoder is working and elevatorMode is true (in automatic mode)
	 * @param inches target height in inches off the floor
	 */
	public void setElevatorPos(double inches) {
		if (elevEncOK && elevatorMode) {
			elevatorMotor1.set(ControlMode.Position, inchesToEncoderTicks(inches - Robot.robotPrefs.elevatorBottomToFloor));
			Robot.log.writeLog("Elevator", "Position set", "Target," + inches);
		}
	}

	/**
	 * Returns the height that elevator is trying to move to in inches from the floor.
	 * Returns -1 if the elevator is in manual mode.
	 * <p><b>NOTE:</b> This is the target height, not the current height.
	 * 
	 * @return desired inches of elevator height
	 */
	public double getCurrentElevatorTarget() {
		if (elevatorMode) {
			return encoderTicksToInches(elevatorMotor1.getClosedLoopTarget(0)) + Robot.robotPrefs.elevatorBottomToFloor;
		} else {
			return -1;
		}
	}

	/**
	 * @return Current elevator position, in inches from floor
	 */
	public double getElevatorPos() {
		return encoderTicksToInches(getElevatorEncTicks()) + Robot.robotPrefs.elevatorBottomToFloor;
	}

	/**
	 * stops elevator motors
	 */
	public void stopElevator() {
		setElevatorMotorPercentOutput(0.0);
	}

	/**
	 * only zeros elevator encoder when it is at the zero position (lower limit)
	 */
	public void checkAndZeroElevatorEnc() {
		if (getElevatorLowerLimit()) {
			stopElevator();			// Make sure Talon PID loop won't move the robot to the last set position when we reset the enocder position
			elevatorMotor1.setSelectedSensorPosition(0, 0, 0);
			Robot.log.writeLog("Elevator", "Zero Encoder", "");
		}
	}

	/**
	 * @return raw encoder ticks (based on encoder zero being at zero position)
	 */
	public double getElevatorEncTicks() {
		return elevatorMotor1.getSelectedSensorPosition(0);
	}

	/**
	 * @param encoderTicks in enocder Ticks
	 * @return parameter encoder ticks converted to equivalent inches
	 */
	public double encoderTicksToInches(double encoderTicks) {
		return (encoderTicks / Robot.robotPrefs.encoderTicksPerRevolution) * (Robot.robotPrefs.elevatorGearCircumference * 2);
	}

	/**
	 * @param inches in inches
	 * @return parameter inches converted to equivalent encoder ticks
	 */
	public double inchesToEncoderTicks(double inches) {
		return (inches / (Robot.robotPrefs.elevatorGearCircumference * 2)) * Robot.robotPrefs.encoderTicksPerRevolution;
	}

	/**
	 * reads whether the elevator is at the upper limit
	 */
	public boolean getElevatorUpperLimit() {
		return elevatorLimits.isFwdLimitSwitchClosed();
	}

	/**
	 * reads whether the elevator is at the lower limit
	 */
	public boolean getElevatorLowerLimit() {
		return elevatorLimits.isRevLimitSwitchClosed();
	}

	/**
	 * returns whether encoder is working or not
	 * @return true is encoder working
	 * 			false is encoder broken
	 */
	public boolean getEncOK() {
		return elevEncOK;
	}

	/**
	 * Checks elevator motor currents, records sticky faults if a motor is faulty for more than 5 cycles
	 */
	public void verifyMotors() {
		double amps1 = Robot.pdp.getCurrent(RobotMap.elevatorMotor1PDP);
		double amps2 = Robot.pdp.getCurrent(RobotMap.elevatorMotor2PDP);

		if(motorFaultCount >= 5) {
			Robot.robotPrefs.recordStickyFaults("Elevator");
			motorFaultCount = 0;
		}

		if(amps1 > 8 && amps2 < 3) {
			motorFaultCount++;
		}
		else if(amps2 > 8 && amps1 < 3) {
			motorFaultCount++;
		}
		else {
			motorFaultCount = 0;
		}
	}

	/**
	 * writes information about the subsystem to the fileLog
	 */
	public void updateElevatorLog() {
		Robot.log.writeLog("Elevator", "Update Variables",
				"Volts1," + elevatorMotor1.getMotorOutputVoltage() + ",Volts2," + elevatorMotor2.getMotorOutputVoltage() + 
				",Amps1," + Robot.pdp.getCurrent(RobotMap.elevatorMotor1PDP) + ",Amps2," + Robot.pdp.getCurrent(RobotMap.elevatorMotor2PDP) + 
				",Enc Ticks," + getElevatorEncTicks() + ",Enc Inches," + getElevatorPos() + 
				",Upper Limit," + getElevatorUpperLimit() + ",Lower Limit," + getElevatorLowerLimit() + 
				",Enc OK," + elevEncOK + ",Elev Mode," + elevatorMode);
	}

	@Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		if (!elevatorMode) {
			setDefaultCommand(new ElevatorWithXBox());
		}
	}

	
	@Override
	public void periodic() {
		
		// Can some of these be eliminated by competition?

		if (Robot.log.getLogRotation() == FileLog.ELEVATOR_CYCLE) {
			SmartDashboard.putBoolean("Elev encOK", elevEncOK);
			SmartDashboard.putBoolean("Elev Mode", elevatorMode); // See below for note on this
			// SmartDashboard.putNumber("EncSnap", encSnapShot);
			// SmartDashboard.putNumber("Enc Now", currEnc);
			SmartDashboard.putNumber("Elev Pos", getElevatorPos());
			// SmartDashboard.putNumber("Enc Tick", getElevatorEncTicks());
			SmartDashboard.putBoolean("Elev Lower Limit", getElevatorLowerLimit());
			SmartDashboard.putBoolean("Elev Upper Limit", getElevatorUpperLimit());
		}		
		
		// Following code changes the frequency of variable logging depending
		// on the set logLevel, Motors are checked every cycle regardless
		if (DriverStation.getInstance().isEnabled()) {

			verifyMotors(); // What is the concrete use for this?  Move to a pit command, instead of live during match?

			if (Robot.log.getLogRotation() == FileLog.ELEVATOR_CYCLE) {
				updateElevatorLog();
			}
		
			// Following code checks whether the encoder is incrementing in the same direction as the 
			// motor is moving and changes control modes based on state of encoder

			/* All of the code below should be gotten rid of. It doesn't speed anything up in competition - the codriver still has to recognize that the encoders are broken
			and the elevator is stalled. This is just more code to run in periodic() */
			// TODO: Delete everything below this

			currEnc = getElevatorEncTicks();
			if (elevatorMotor1.getMotorOutputVoltage() > 5) {
				if (posMoveCount == 0) {
					encSnapShot = getElevatorEncTicks();
				}
				negMoveCount = 0;
				posMoveCount++;
				if (posMoveCount > 3) {
					elevEncOK = (currEnc - encSnapShot) > 100;
					if (!elevEncOK) {
						Robot.robotPrefs.recordStickyFaults("Elevator Enc");
						setDefaultCommand(new ElevatorWithXBox()); 
						// We can probably ignore the automatic switch to xbox. By the time the codriver realizes, they can just push the joystick button in anyways.
						elevatorMode = false;
					}
					posMoveCount = 0;
				}
			} else if (elevatorMotor1.getMotorOutputVoltage() < -5) {
				if (negMoveCount == 0) {
					encSnapShot = getElevatorEncTicks();
				}
				posMoveCount = 0;
				negMoveCount++;
				if (negMoveCount > 3) {
					elevEncOK = (currEnc - encSnapShot) < -100;
					if (!elevEncOK) {
						Robot.robotPrefs.recordStickyFaults("Elevator Enc");
						setDefaultCommand(new ElevatorWithXBox());
						// We can probably ignore the automatic switch to xbox. By the time the codriver realizes, they can just push the joystick button in anyways.
						elevatorMode = false;
					}
					negMoveCount = 0;
				}
			}
			if (!elevatorMode && elevEncOK) {
				if (getElevatorLowerLimit() && getElevatorEncTicks() == 0) {
					setDefaultCommand(null);
					elevatorMode = true;
					posMoveCount = 0;
					negMoveCount = 0;
				}
			}

		}
		
	}
}