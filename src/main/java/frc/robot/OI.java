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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.commands.*;
import frc.robot.utilities.RobotPreferences.ElevatorPosition;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */

public class OI {

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

    SmartDashboard.putData("LoadToRocketPT1", new DrivePathfinder("RLoadToRocketPT1-A", true, false));
    SmartDashboard.putData("LoadToRocketPT2-2", new DrivePathfinder("RLoadToRocketPT2-A", false, true));
    SmartDashboard.putData("LoadToRocket", new PathfinderLoadToRocket());
    // SmartDashboard.putData("Turn Gyro 90", new TurnGyro(90));
    // SmartDashboard.putData("LoadToRocket", new PathfinderLoadToRocket());
    // The conditional logic needs to go in the command itself. No logic can be done in OI since OI is constructed at the start and not run repeatedly
    // SmartDashboard.putData("Pathfinder Test 1", new DrivePathfinder("Test", true));

    xBoxA.whenPressed(new ElevatorMoveSafe(ElevatorPosition.hatchLow));
    xBoxB.whenPressed(new ElevatorMoveSafe(ElevatorPosition.hatchMid));
    xBoxY.whenPressed(new ElevatorMoveSafe(ElevatorPosition.hatchHigh));
    xBoxX.whenPressed(new ElevatorMoveSafe(ElevatorPosition.cargoShipCargo));
    
    SmartDashboard.putData("Pathfinder Test 1", new DrivePathfinder("Test", true, true));

    // Buttons for controlling the elevator
    SmartDashboard.putData("Elevator Up", new ElevatorRaise()); // For testing limit switch and encoder
    SmartDashboard.putData("Elevator Down", new ElevatorLower()); // For testing limit switch and encoder
    SmartDashboard.putData("Move Elevator to Bottom", new ElevatorMoveToLevel(ElevatorPosition.bottom)); // Move to encoder's zero position
    SmartDashboard.putData("Move Elevator to WristStow", new ElevatorMoveToLevel(ElevatorPosition.wristStow)); // Move to level that wrist can be stowed safely
    SmartDashboard.putData("Move Elevator to High", new ElevatorMoveToLevel(ElevatorPosition.hatchHigh)); // Move to high position (test wrist interlock)
    SmartDashboard.putData("Zero Elev Enc (w/ Limit)", new ElevatorMoveToBottomThenZeroEncoder());

    // Buttons for controlling the climber
    SmartDashboard.putData("Climb Up", new ClimbArmSetPercentOutput(0.2));  // For testing
    SmartDashboard.putData("Climb Down", new ClimbArmSetPercentOutput(-0.2));  // For testing
    SmartDashboard.putData("Climb move to start", new ClimbArmStow());  // For testing
    SmartDashboard.putData("Climb move to vacuum", new ClimbArmSetAngle(Robot.robotPrefs.climbVacuumAngle));  // For testing
    SmartDashboard.putData("Climb lift robot", new ClimbArmSetAngle(Robot.robotPrefs.climbLiftAngle));  // For testing
    SmartDashboard.putData("Climb Vacuum On", new ClimbVacuumTurnOn(true));
    SmartDashboard.putData("Climb Vacuum Off", new ClimbVacuumTurnOn(false));
    SmartDashboard.putData("Climb Set Reference", new ClimbMoveToLimitThenCalibrate());
    SmartDashboard.putData("ClimbMoveUntilVacuum", new ClimbMoveUntilVacuum(Robot.robotPrefs.climbVacuumAngle));
    SmartDashboard.putData("ClimbSequnce", new ClimbSequence());

    // Buttons for the Cargo rollers
    SmartDashboard.putData("Cargo Intake", new CargoIntake());
    SmartDashboard.putData("Cargo Outtake", new CargoOuttake());

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
  }

  public void setDriveDirection(boolean direction) {
    this.driveDirection = direction;
  }

  public boolean getDriveDirection() {
    return driveDirection;
  }
}
