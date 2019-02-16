/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.commands.*;
import frc.robot.triggers.*;
import frc.robot.utilities.RobotPreferences;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */

public class OI {
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());

  public Joystick leftJoystick = new Joystick(0);
  public Joystick rightJoystick = new Joystick(1);
  public Joystick coPanel = new Joystick(2);
  public XboxController xBoxController = new XboxController(3);
  private boolean driveDirection = true;

  private Button xBoxA = new JoystickButton(xBoxController, 1);
  private Button xBoxB = new JoystickButton(xBoxController, 2);
  private Button xBoxX = new JoystickButton(xBoxController, 3);
  private Button xBoxY = new JoystickButton(xBoxController, 4);

  private Trigger trigWristElevEncoder = new WristEncoderCheck();

  public OI() {
    Button[] left = new Button[12];
    Button[] right = new Button[12];
    Button[] coP = new Button[15];
    Button[] xbB = new Button[10];
    
    for (int i = 1; i < left.length; i++) {
      left[i] = new JoystickButton(leftJoystick, i);
      right[i] = new JoystickButton(rightJoystick, i);

      if (i == 1) {
        left[i].whenPressed(new Shift(false));
        right[i].whenPressed(new Shift(true));
      } else if (i == 2) {
        left[i].whenPressed(new DriveAssist());
        right[i].whenPressed(new DriveAssist());
        left[i].whenReleased(new DriveWithJoysticks());
        right[i].whenReleased(new DriveWithJoysticks());
      } else if (i == 3) {
        left[i].whenPressed(new LEDSetColor(0));
        left[i].whenReleased(new DriveWithJoysticks());
      } else if (i == 4){
        left[i].whenPressed(new LEDSetColor(1));
        left[i].whenPressed(new DriveWithJoysticks());
      } else if (i == 5){
        left[i].whenPressed(new LEDSetColor(2));
        left[i].whenPressed(new DriveWithJoysticks());
      } else if (i == 6){
        left[i].whenPressed(new LEDSetColor(3));
        left[i].whenPressed(new DriveWithJoysticks());
      } else if (i == 11){
        left[i].whenPressed(new LEDBlink(1));
        left[i].whenPressed(new DriveWithJoysticks());
      } else if (i == 10){
        left[i].whenPressed(new LEDBlink(2));
        left[i].whenPressed(new DriveWithJoysticks());
      }
    }

    SmartDashboard.putData("LoadToRocketPT1", new DrivePathfinder("RLoadToRocketPT1-A", true, false));
    SmartDashboard.putData("LoadToRocketPT2-2", new DrivePathfinder("RLoadToRocketPT2-A", false, true));
    SmartDashboard.putData("LoadToRocket", new PathfinderLoadToRocket());
    // SmartDashboard.putData("Turn Gyro 90", new TurnGyro(90));
    // SmartDashboard.putData("LoadToRocket", new PathfinderLoadToRocket());
    // The conditional logic needs to go in the command itself. No logic can be done in OI since OI is constructed at the start and not run repeatedly
    // SmartDashboard.putData("Pathfinder Test 1", new DrivePathfinder("Test", true));

    xBoxA.whenActive(new ElevatorMoveToLevel(RobotPreferences.ElevatorPosition.hatchLow));
    xBoxB.whenActive(new ElevatorMoveToLevel(RobotPreferences.ElevatorPosition.hatchMid));
    xBoxY.whenActive(new ElevatorMoveToLevel(RobotPreferences.ElevatorPosition.hatchHigh));
    xBoxX.whenActive(new ElevatorMoveToLevel(RobotPreferences.ElevatorPosition.cargoShipCargo));
    
    SmartDashboard.putData("Pathfinder Test 1", new DrivePathfinder("Test", true, true));
    SmartDashboard.putData("Turn To Target", new VisionTurnToTarget());
    SmartDashboard.putData("Drive on line", new DriveWithLineFollowing());

    // Buttons for controlling the elevator
    SmartDashboard.putData("Elevator Up", new ElevatorRaise()); // For testing limit switch and encoder
    SmartDashboard.putData("Elevator Down", new ElevatorLower()); // For testing limit switch and encoder
    SmartDashboard.putData("Move Elevator to Bottom", new ElevatorMoveToLevel(RobotPreferences.ElevatorPosition.bottom)); // Move to encoder's zero position
    SmartDashboard.putData("Move Elevator to WristSafe", new ElevatorMoveToLevel(RobotPreferences.ElevatorPosition.wristSafe)); // Move to encoder's zero position
    SmartDashboard.putData("Zero Elev Enc (w/ Limit)", new ElevatorEncoderZero());

    // Buttons for controlling FileLogging
    SmartDashboard.putData("Log 1 InitialTesting", new FileLogSetLevel(1));
    SmartDashboard.putData("Log 2 PitTesting", new FileLogSetLevel(2));
    SmartDashboard.putData("Log 3 Competition", new FileLogSetLevel(3));
    Robot.log.setLogLevel(3);   // Also puts log level indicator on ShuffleBoard

    SmartDashboard.putBoolean("Left LineFollower", Robot.lineFollowing.isLinePresent(1));
    SmartDashboard.putBoolean("Middle LineFollower", Robot.lineFollowing.isLinePresent(2));
    SmartDashboard.putBoolean("Right LineFollower", Robot.lineFollowing.isLinePresent(3));
    
    SmartDashboard.putData("Clear Sticky Faults", new ClearStickyFaults());
    Robot.robotPrefs.showStickyFaults();
    //SmartDashboard.putData("Turn To Line", new TurnToLine());
    SmartDashboard.putData("Disc Toggle", new HatchToggle());
    SmartDashboard.putData("Disc Grab", new HatchSet(true));
    SmartDashboard.putData("Disc Release", new HatchSet(false));
    SmartDashboard.putString("Disc Position", "Null");

    SmartDashboard.putData("LEDSet Purple", new LEDSetColor(0));
    SmartDashboard.putData("LEDSet Red", new LEDSetColor(1));
    SmartDashboard.putData("LEDSet Blue", new LEDSetColor(2));
    SmartDashboard.putData("LEDSet Off", new LEDSetColor(3));

  }

  public void setDriveDirection(boolean direction) {
    this.driveDirection = direction;
  }

  public boolean getDriveDirection() {
    return driveDirection;
  }
}
