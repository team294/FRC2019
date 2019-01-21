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
  public static final double encoderTicksPerRevolution = 4096.0;

  // Pneumatic Addresses
  public static final int pnuematicShifterLow = 0;
  public static final int pnuematicShifterHigh = 1;
  
  // RoboRIO digital I/O Addresses
  
  //RoboRIO digital I/O Addresses
  public static int lineFollowerLeft = 2;
  public static int lineFollowerCenter = 1;
  public static int lineFollowerRight = 0;


  // PDP Addresses
  public static final int leftMotor1PDP = 13; //Check on PDP and change
  public static final int leftMotor2PDP = 15; //Check on PDP and change
  public static final int leftMotor3PDP = 14; //Check on PDP and change
  public static final int rightMotor1PDP = 2; //Check on PDP and change
  public static final int rightMotor2PDP = 0; //Check on PDP and change
  public static final int rightMotor3PDP = 1; //Check on PDP and change 
}
