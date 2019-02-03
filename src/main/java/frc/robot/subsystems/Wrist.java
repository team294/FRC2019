/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Wrist extends Subsystem {
  private WPI_TalonSRX wristMotor = new WPI_TalonSRX(RobotMap.wristMotor);

  public Wrist() {
    wristMotor.set(ControlMode.PercentOutput, 0);
    wristMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
    wristMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    // wristMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    wristMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void zeroWristEncoder() {
    wristMotor.setSelectedSensorPosition(0, 0, 0);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
