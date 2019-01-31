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
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.commands.*;
import frc.robot.subsystems.Elevator;

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
      } else if (i == 3) {
        left[i].whenPressed(new DriveWithVision());
        left[i].whenReleased(new DriveWithJoysticks());
        right[i].whenPressed(new VisionTurnToTarget());
        right[i].whenReleased(new DriveWithJoysticks()); // We should be able to cancel the commands when the button is released. This is a better method to do that.
      }
    }

    SmartDashboard.putData("Pathfinder Test 1", new DrivePathfinder("Test", true));

    /*
    if (isBall) { //TODO uncomment when the sensor that tells whether we have a ball or hatch is added
    xBoxA.whenActive(new ElevatorMoveToLevel(RobotMap.HatchLow + RobotMap.ballOffset));
    xBoxB.whenActive(new ElevatorMoveToLevel(RobotMap.HatchMid + RobotMap.ballOffset));
    xBoxY.whenActive(new ElevatorMoveToLevel(RobotMap.HatchHigh + RobotMap.ballOffset));
    xBoxX.whenActive(new ElevatorMoveToLevel(RobotMap.CargoShipCargo));
    }
    else {
      */
    xBoxA.whenActive(new ElevatorMoveToLevel(RobotMap.HatchLow));
    xBoxB.whenActive(new ElevatorMoveToLevel(RobotMap.HatchMid));
    xBoxY.whenActive(new ElevatorMoveToLevel(RobotMap.HatchHigh));
    xBoxX.whenActive(new ElevatorMoveToLevel(RobotMap.CargoShipCargo));
    
    SmartDashboard.putData("Turn To Target", new VisionTurnToTarget());
    SmartDashboard.putData("Elevator F", new ElevatorForward()); // For testing limit switch
    SmartDashboard.putData("Elevator R", new ElevatorReverse()); // For testing limit switch
    SmartDashboard.putData("Calib", new ElevatorMoveToLevel(10.0));
    SmartDashboard.putData("Move Zero", new ElevatorMoveToLevel(0.0));
    SmartDashboard.putData("Zero Elev Enc (w/ Limit)", new ElevatorEncoderZero());
    SmartDashboard.putData("Manual Zero Elev Enc (w/out Limit)", new ElevatorManualZero());
    SmartDashboard.putData("Turn To Line", new TurnToLine());
    SmartDashboard.putBoolean("Left LineFollower", Robot.lineFollowing.isLinePresent(1));
    SmartDashboard.putBoolean("Middle LineFollower", Robot.lineFollowing.isLinePresent(2));
    SmartDashboard.putBoolean("Right LineFollower", Robot.lineFollowing.isLinePresent(3));
  }

  public void setDriveDirection(boolean direction) {
    this.driveDirection = direction;
  }

  public boolean getDriveDirection() {
    return driveDirection;
  }
}
