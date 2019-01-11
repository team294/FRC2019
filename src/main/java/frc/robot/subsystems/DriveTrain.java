/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;
import frc.robot.commands.DriveWithJoysticks;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Drive Train subsystem.  
 */
public class DriveTrain extends Subsystem {
  // This is to test with the 2018 drive base.  The 2019 drive base will use 4 Talon SBX controllers for follower motors 1 and 3
  private final WPI_TalonSRX leftMotor1 = new WPI_TalonSRX(RobotMap.leftMotor1);
  private final WPI_TalonSRX leftMotor2 = new WPI_TalonSRX(RobotMap.leftMotor2);
  private final WPI_TalonSRX leftMotor3 = new WPI_TalonSRX(RobotMap.leftMotor3);
  private final WPI_TalonSRX rightMotor1 = new WPI_TalonSRX(RobotMap.rightMotor1);
  private final WPI_TalonSRX rightMotor2 = new WPI_TalonSRX(RobotMap.rightMotor2);
  private final WPI_TalonSRX rightMotor3 = new WPI_TalonSRX(RobotMap.rightMotor3);

  public final DifferentialDrive robotDrive = new DifferentialDrive(leftMotor2, rightMotor2);

  public DriveTrain() {
    leftMotor1.set(ControlMode.Follower, RobotMap.leftMotor2);
    leftMotor3.set(ControlMode.Follower, RobotMap.leftMotor2);
    rightMotor1.set(ControlMode.Follower, RobotMap.rightMotor2);
    rightMotor3.set(ControlMode.Follower, RobotMap.rightMotor2);
    leftMotor2.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
    rightMotor2.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);

    leftMotor2.clearStickyFaults(0);
    rightMotor2.clearStickyFaults(0);

    leftMotor2.setNeutralMode(NeutralMode.Brake);
    rightMotor2.setNeutralMode(NeutralMode.Brake);
  }

  public void tankDrive (double powerLeft, double powerRight) {
    this.robotDrive.tankDrive(powerLeft, powerRight);
  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new DriveWithJoysticks());
  }
}
