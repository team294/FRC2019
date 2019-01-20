/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Utilities.*;
import frc.robot.commands.ElevatorWithJoysticks;
import frc.robot.commands.StopElevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;

/**
 * Add your docs here.
 */
public class Elevator extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.  

  private CANSparkMax elevatorMotor1;
  private CANSparkMax elevatorMotor2;
  private CANEncoder elevatorEncoder1;
  private CANEncoder elevatorEncoder2;

  private int periodicCount = 0;

  private double encoder1Zero = 0;
  private double encoder2Zero = 0;

  public Elevator() {
	elevatorMotor1 = new CANSparkMax(RobotMap.elevatorMotor1, MotorType.kBrushless);
	elevatorMotor2 = new CANSparkMax(RobotMap.elevatorMotor2, MotorType.kBrushless);
	elevatorEncoder1 = elevatorMotor1.getEncoder();
	elevatorEncoder2 = elevatorMotor2.getEncoder();
	elevatorMotor1.clearFaults();	
	elevatorMotor2.clearFaults();
	zeroEncoder1();
	zeroEncoder2();	
	}
	
	/* 
	powerLeft and powerRight are between -1.0 and 1.0
	*/
	public void setElevatorMotorPercentPower(double percentPower) {
		elevatorMotor1.set(percentPower);
		elevatorMotor2.set(percentPower);
	}

	public void stopElevator() {
		setElevatorMotorPercentPower(0.0);
	}

	public void zeroEncoder1() {
		encoder1Zero = elevatorEncoder1.getPosition();
	}

	public double getZero() {
		return encoder1Zero;
	}

	public void zeroEncoder2() {
		encoder2Zero = elevatorEncoder2.getPosition();
	}

	public double getEncoder1Revolutions() {
		return elevatorEncoder1.getPosition() - encoder1Zero;
	}

	public double getEncoder2Revolutions() {
		return elevatorEncoder2.getPosition() - encoder2Zero;
	}

	public double encoderRevolutionsToInches(double encoderRevs) {
		return encoderRevs * Robot.robotPrefs.elevatorGearCircumference;
	}

	public double inchesToEncoderRevolutions(double inches) {
		return inches / Robot.robotPrefs.elevatorGearCircumference;
	}

	public double getEncoder1Inches() {
		return encoderRevolutionsToInches(getEncoder1Revolutions());
	}

	public double getEncoder2Inches() {
		return encoderRevolutionsToInches(getEncoder2Revolutions());
	}

	public void updateElevatorLog() {
		Robot.log.writeLog("Elevator", "Update Variables",
		"Elev1 Volts," + elevatorMotor1.getBusVoltage() + ",Elev2 Volts," + elevatorMotor2.getBusVoltage() +
		",Elev1 Amps," + elevatorMotor1.getOutputCurrent() + ",Elev2 Amps," + elevatorMotor2.getOutputCurrent() +
		",Elev1 Temp," + elevatorMotor1.getMotorTemperature() + ",Elev2 Temp," + elevatorMotor2.getMotorTemperature() +
		",Elev Enc1 Revs," + getEncoder1Revolutions() + ",Elev Enc2 Revs," + getEncoder2Revolutions() +
		",Elev Enc1 Inches," + getEncoder1Inches() + ",Elev Enc2 Inches," + getEncoder2Inches());
	}
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
	setDefaultCommand(new StopElevator());
  }

  @Override
  public void periodic() {

    if (DriverStation.getInstance().isEnabled()) {
      if ((++periodicCount) >= 25) {
        updateElevatorLog();
        periodicCount=0;  
      }
    }
  }
}
