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
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
  public static final int leftMotor1 = 10;
  public static final int leftMotor2 = 11;
  public static final int leftMotor3 = 12;
  public static final int rightMotor1 = 20;
  public static final int rightMotor2 = 21;
  public static final int rightMotor3 = 22;
  public static final double encoderTicksPerRevolution = 4096.0;

  // Pneumatic Addresses
  public static final int pnuematicShifterLow = 0;
  public static final int pnuematicShifterHigh = 1;
  
  // RoboRIO digital I/O Addresses
  public static int lineFollower1 = 0;
  public static int lineFollower2 = 1;
  public static int lineFollower3 = 2;

  // TODO Update with 2018 base
  // Imperial versions
  public static final double wheelbase_in = 25.0;       // wheelbase, in inches
  public static final double wheel_diameter_in = 6.0;   // wheel diamater, in inches
  public static final double wheel_distance_in_per_tick = wheel_diameter_in*Math.PI/encoderTicksPerRevolution;  // wheel distance traveled per encoder tick, in inches
  public static final double max_velocity_ips = 165.0;   // max robot velocity, in inches per second
  public static final double max_acceleration_ipsps = 80.0;  // max robot acceleration, in inches per second per second
  public static final double max_jerk_ipspsps = 2400.0;  // max robot jerk, in inches per second per second per second
  
  // TODO Update with 2018 base
  // Metric versions
  public static final double wheelbase_m = wheelbase_in*0.0254;           // wheelbase, in meters
  public static final double wheel_diameter_m = wheel_diameter_in*0.0254; // wheel diamater, in meters
  public static final double wheel_distance_m_per_tick = wheel_diameter_in*0.0254;  // wheel distance traveled per encoder tick, in meters
  public static final double max_velocity_mps = max_velocity_ips*0.0254;  // max robot velocity, in meters per second
  public static final double max_acceleration_mpsps = max_acceleration_ipsps*0.0254;  // max robot acceleration, in meters per second per second
  public static final double max_jerk_mpspsps = max_jerk_ipspsps*0.0254;  // max robot jerk, in meters per second per second per second

}
