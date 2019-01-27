/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  
  public static final int PowerDistributionPanel = 0;

  public static final int leftMotor1 = 10;
  public static final int leftMotor2 = 11;
  public static final int leftMotor3 = 12;
  public static final int rightMotor1 = 20;
  public static final int rightMotor2 = 21;
  public static final int rightMotor3 = 22;
  public static final int elevatorMotor1 = 30;
  public static final int elevatorMotor2 = 31;
  public static final double encoderTicksPerRevolution = 4096.0;

  // Pneumatic Addresses
  public static final int pnuematicShifterLow = 0;
  public static final int pnuematicShifterHigh = 1;
  
  // TODO Update with 2018 base
  // Imperial versions
  public static final double wheelbase_in = 25.0;       // wheelbase, in inches
  public static final double wheel_diameter_in = 6.0;   // wheel diamater, in inches
  public static final double wheel_distance_in_per_tick = wheel_diameter_in*Math.PI/encoderTicksPerRevolution;  // wheel distance traveled per encoder tick, in inches
  public static final double max_velocity_ips = 200.0;   // max robot velocity, in inches per second
  public static final double max_acceleration_ipsps = 130.0;  // max robot acceleration, in inches per second per second
  public static final double max_jerk_ipspsps = 2400.0;  // max robot jerk, in inches per second per second per second
  
  // TODO Update with 2018 base
  // Metric versions
  public static final double wheelbase_m = wheelbase_in*0.0254;           // wheelbase, in meters
  public static final double wheel_diameter_m = wheel_diameter_in*0.0254; // wheel diamater, in meters
  public static final double wheel_distance_m_per_tick = wheel_diameter_in*0.0254;  // wheel distance traveled per encoder tick, in meters
  public static final double max_velocity_mps = max_velocity_ips*0.0254;  // max robot velocity, in meters per second
  public static final double max_acceleration_mpsps = max_acceleration_ipsps*0.0254;  // max robot acceleration, in meters per second per second
  public static final double max_jerk_mpspsps = max_jerk_ipspsps*0.0254;  // max robot jerk, in meters per second per second per second

  public static final int lineFollower1 = 0;
  public static final int lineFollower2 = 1;
  public static final int lineFollower3 = 2;
  public static final int elevatorLowerLimit = 4;

  // PDP Addresses
  public static final int leftMotor1PDP = 13; //Check on PDP and change
  public static final int leftMotor2PDP = 15; //Check on PDP and change
  public static final int leftMotor3PDP = 14; //Check on PDP and change
  public static final int rightMotor1PDP = 2; //Check on PDP and change
  public static final int rightMotor2PDP = 0; //Check on PDP and change
  public static final int rightMotor3PDP = 1; //Check on PDP and change
  public static final int elevatorMotor1PDP = 0; //Check on PDP and change
  public static final int elevatorMotor2PDP = 0; //Check on PDP and change

  //Elevator Level Heights (in inches)
  public static final double HatchLow = 19.0 - Robot.robotPrefs.robotOffset;
  public static final double HatchMid = 47.0 - Robot.robotPrefs.robotOffset;
  public static final double HatchHigh = 75.0 - Robot.robotPrefs.robotOffset;
  public static final double CargoShipCargo = 34.75 - Robot.robotPrefs.robotOffset;
  public static final double ballOffset = 8.5;

}
