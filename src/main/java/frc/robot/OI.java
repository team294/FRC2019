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
  public Joystick xBoxController = new Joystick(3);
  private boolean driveDirection = true;

  private Trigger trigWristElevEncoder = new WristEncoderCheck();

  public OI() {
    Button[] left = new Button[12];
    Button[] right = new Button[12];
    Button[] coP = new Button[15];
    Button[] xbB = new Button[10];
    Trigger xbUp = new POVTrigger(xBoxController, 0);
    Trigger xbRight = new POVTrigger(xBoxController, 90);
    Trigger xbDown = new POVTrigger(xBoxController, 180);
    Trigger xbLeft = new POVTrigger(xBoxController, 270);
    Trigger xbLT = new AxisTrigger(xBoxController, 2, 0.9);
    Trigger xbRT = new AxisTrigger(xBoxController, 3, 0.9);
    
    for (int i = 1; i < left.length; i++) {
      left[i] = new JoystickButton(leftJoystick, i);
      right[i] = new JoystickButton(rightJoystick, i);

      if (i == 1) {
        left[i].whenPressed(new Shift(true));
        right[i].whenPressed(new Shift(false));
      } else if (i == 2) {
        left[i].whenPressed(new DriveAssist());
        right[i].whenPressed(new DriveAssist());
        left[i].whenReleased(new DriveWithJoysticks());
        right[i].whenReleased(new DriveWithJoysticks());
      } else if (i == 3) {
        left[i].whenPressed(new DriveWithVision(false, true)); // No line followers, but gyro correction
        left[i].whenReleased(new DriveWithJoysticks());
        right[i].whenPressed(new DriveWithVision(false, false)); // No line followers, no gyro
        right[i].whenReleased(new DriveWithJoysticks());
      } else if (i == 11 || i == 10) {
        left[i].whenPressed(new DriveWithLineFollowing(true));
        left[i].whenReleased(new DriveWithJoysticks());
        right[i].whenPressed(new DriveWithLineFollowing(false));
        right[i].whenReleased(new DriveWithJoysticks());
      }
    }

    for (int i = 1; i < xbB.length; i++) {
      xbB[i] = new JoystickButton(xBoxController, i);
    }

    xbB[1].whenPressed(new ElevatorMoveAndScore(RobotPreferences.ElevatorPosition.hatchLow));
    xbB[2].whenPressed(new ElevatorMoveAndScore(RobotPreferences.ElevatorPosition.hatchMid));
    xbB[3].whenPressed(new ElevatorMoveAndScore(RobotPreferences.ElevatorPosition.hatchHigh));
    xbB[4].whenPressed(new ElevatorWristStow());
    xbB[5].whenPressed(new CargoStop());
    xbB[6].whenPressed(new CargoStop());
    // xbB[7].whenPressed(new Command());
    // xbB[8].whenPressed(new Command());
    xbB[9].toggleWhenPressed(new ElevatorWithXBox());
    xbB[10].toggleWhenPressed(new WristWithXBox());
    xbUp.whenActive(new CargoIntakeFromLoad());
    xbRight.whenActive(new HatchSet(false));
    xbDown.whenActive(new CargoIntakeFromGround());
    xbLeft.whenActive(new HatchSet(true));
    xbLT.whenActive(new CargoOuttake());
    xbRT.whenActive(new CargoOuttake());

    // Buttons for controlling the elevator
    SmartDashboard.putData("Elevator Up", new ElevatorRaise()); // For testing limit switch and encoder
    SmartDashboard.putData("Elevator Down", new ElevatorLower()); // For testing limit switch and encoder
    SmartDashboard.putData("Move Elevator to Bottom", new ElevatorMoveToLevel(RobotPreferences.ElevatorPosition.bottom)); // Move to encoder's zero position
    SmartDashboard.putData("Move Elevator to WristSafe", new ElevatorMoveToLevel(RobotPreferences.ElevatorPosition.wristSafe)); // Move to encoder's zero position
    SmartDashboard.putData("Zero Elev Enc (w/ Limit)", new ElevatorEncoderZero());

    // Buttons for controlling the climber
    SmartDashboard.putData("Climb Up", new ClimbArmSetPercentOutput(0.2));  // For testing
    SmartDashboard.putData("Climb Down", new ClimbArmSetPercentOutput(-0.2));  // For testing
    SmartDashboard.putData("Climb move to 0", new ClimbArmSetAngle(0));  // For testing
    SmartDashboard.putData("Climb move to start", new ClimbArmSetAngle(Robot.robotPrefs.climbStartingAngle + 5));  // For testing
    SmartDashboard.putData("Climb Vacuum On", new ClimbVacuumTurnOn(true));
    SmartDashboard.putData("Climb Vacuum Off", new ClimbVacuumTurnOn(false));
    SmartDashboard.putData("Climb Set Reference", new ClimbEncoderCalibrateAtLimit());
    SmartDashboard.putData("ClimbMoveUntilVacuum", new ClimbMoveUntilVacuum(Robot.robotPrefs.climbVacuumAngle));
    SmartDashboard.putData("ClimbLiftRobot", new ClimbLiftRobot(Robot.robotPrefs.climbLiftAngle));
    SmartDashboard.putData("ClimbSequnce", new ClimbSequence());

    // Buttons for controlling FileLogging
    SmartDashboard.putData("Log 1 InitialTesting", new FileLogSetLevel(1));
    SmartDashboard.putData("Log 2 PitTesting", new FileLogSetLevel(2));
    SmartDashboard.putData("Log 3 Competition", new FileLogSetLevel(3));
    Robot.log.setLogLevel(3);   // Also puts log level indicator on ShuffleBoard

    // These aren't useful; they don't update when the robot is running
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
    SmartDashboard.putData("Pathfinder Test 1", new DrivePathfinder("Test", true, true));
  }

  public void setDriveDirection(boolean direction) {
    this.driveDirection = direction;
  }

  public boolean getDriveDirection() {
    return driveDirection;
  }
}
