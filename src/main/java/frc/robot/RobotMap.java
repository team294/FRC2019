
package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  
  // CAN bus addresses
  public static final int powerDistributionPanel = 0;

  public static final int leftMotor1 = 10;
  public static final int leftMotor2 = 11;       // Talon
  public static final int leftMotor3 = 12;
  public static final int rightMotor1 = 20;
  public static final int rightMotor2 = 21;      // Talon
  public static final int rightMotor3 = 22;
  
  public static final int elevatorMotor1 = 30;   // Talon
  public static final int elevatorMotor2 = 31;   // Talon

  public static final int cargoMotor1 = 40;
  public static final int cargoMotor2 = 41;

  public static final int climbMotor1 = 50;     // Talon
  public static final int climbMotor2 = 51;     // Talon
  public static final int climbVacuum1 = 52;
  public static final int climbVacuum2 = 53;

  public static final int wristMotor = 60;      // Talon

  // Pneumatic Addresses
  public static final int pnuematicShifterLow = 0;
  public static final int pnuematicShifterHigh = 1;
  public static final int pneumaticHatchIn = 2;
  public static final int pneumaticHatchOut = 3;
 
  //RoboRIO digital I/O Addresses
  public static final int lineFollowerLeft = 2;
  public static final int lineFollowerCenter = 1;
  public static final int lineFollowerRight = 0;
  public static final int vacuumSwitch = 3;
  public static final int elevatorLowerLimit = 4;
  
  // PDP Addresses
  public static final int leftMotor1PDP = 0; 
  public static final int leftMotor2PDP = 1; 
  public static final int leftMotor3PDP = 2; 
  public static final int rightMotor1PDP = 15; 
  public static final int rightMotor2PDP = 14; 
  public static final int rightMotor3PDP = 13; 
  public static final int climbMotor1PDP = 3; 
  public static final int climbMotor2PDP = 12;  
  public static final int climbVacuum1PDP = 10; 
  public static final int climbVacuum2PDP = 9; 
  public static final int elevatorMotor1PDP = 11; 
  public static final int elevatorMotor2PDP = 4; 
  public static final int cargoMotor1PDP = 6;  
  public static final int cargoMotor2PDP = 7;  
  public static final int wristMotorPDP = 5;  
}
