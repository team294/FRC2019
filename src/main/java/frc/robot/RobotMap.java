
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
  public static final int leftMotor2 = 11;       // Talon with left drive encoder
  public static final int leftMotor3 = 12;
  public static final int rightMotor1 = 20;
  public static final int rightMotor2 = 21;      // Talon with right drive encoder
  public static final int rightMotor3 = 22;
  
  public static final int elevatorMotor1 = 30;   // Talon with elevator encoder and UL/LL switches
  public static final int elevatorMotor2 = 31;   // Talon (with left drive encoder in Spark config)

  public static final int cargoMotor1 = 40;      // Top cargo motor
  // public static final int cargoMotor2 = 41;      // Bottom cargo motor
  public static final int rearHatchMotor = 41;       // Talon (was bottom cargo motor)

  public static final int climbMotor1 = 50;      // Talon (with right drive encoder in Spark config)
  public static final int climbMotor2 = 51;      // Talon with climb encoder and UL switch
  public static final int climbVacuum1 = 52;
  // public static final int climbVacuum2 = 53;

  public static final int wristMotor = 60;       // Talon with wrist encoder and UL/LL switches

  // Pneumatic Addresses
  // public static final int pnuematicShifterLow = 0;
  public static final int pnuematicShifterHigh = 0;   // Changed to single ended Default Low
  public static final int pneumaticHatch = 1;
  public static final int pneumaticRearHatch = 3; // not used
  public static final int hatchExtensionPiston = 2;
  
  public static final int pneumaticLedsBlue = 4;
  public static final int pneumaticLedsRed = 5;
  public static final int pneumaticLedsGreen = 6;
 
  // RoboRIO digital I/O Addresses
  public static final int lineFollowerLeft = 2;
  public static final int lineFollowerCenter = 1;
  public static final int lineFollowerRight = 0;
  // public static final int vacuumSwitch = 3;  // Gnd (black) = "C" common, White (sense) = "NC" normally closed
  public static final int photoSwitchCargo = 4;

  // Analog Inputs
  public static final int analogVacuum = 2;

  // Relay Addresses
  public static final int ledRelay =0;
  
  //CAN Pneumatic Control Module
  public static final int ledPCM = 0;
  
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
  // public static final int climbVacuum2PDP = 9; 
  public static final int elevatorMotor1PDP = 11; 
  public static final int elevatorMotor2PDP = 4; 
  public static final int cargoMotor1PDP = 6;  
  public static final int cargoMotor2PDP = 7;  
  public static final int wristMotorPDP = 5;
  public static final int rearHatchMotorPDP = 9;

}
