/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.RobotMap;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.command.Subsystem;

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


}
