/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utilities.*;
import frc.robot.commands.VisionSandstormSetup;
import frc.robot.subsystems.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static final String ClimbPressure = null;
  public static DriveTrain driveTrain;
  public static Shifter shifter;
  public static Elevator elevator;
  public static Wrist wrist;
  public static Cargo cargo;
  public static Hatch hatch;
  public static VisionData vision;
  public static LineFollowing lineFollowing;
  public static Climb climb;
  public static OI oi;
  public static FileLog log;
  public static RobotPreferences robotPrefs;
  public static PowerDistributionPanel pdp;
  public static LedHandler leds;

  public static boolean beforeFirstEnable = true; // true before the first time the robot is enabled after loading code
  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    // Create file log first, so any other class constructors can log data
    log = new FileLog("A3");

    // Read robot preference next, so any other class constructors can use preferences 
    robotPrefs = new RobotPreferences();
    robotPrefs.doExist();   // Sets up Robot Preferences if they do not exist : ie you just replaced RoboRio
    
    beforeFirstEnable = true; // set variable that robot has not been enabled

    // Create all subsystems and utilities
    driveTrain = new DriveTrain();
    shifter = new Shifter();
    elevator = new Elevator();
    wrist = new Wrist();
    cargo = new Cargo();
    hatch = new Hatch();
    vision = new VisionData();
    lineFollowing = new LineFollowing();
    climb = new Climb();
    leds = new LedHandler();
    pdp = new PowerDistributionPanel();
    // pdp.clearStickyFaults();
    // m_chooser.setDefaultOption("Default Auto", new ExampleCommand());
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);
    climb.enableCompressor(true);

    // Create OI last, so all subsystem and utility objects are created before OI
    oi = new OI();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    log.advanceLogRotation(); // This moves the log up by one every period tick, which other subsystems' periodic functions reference

    if (Robot.log.getLogRotation() == FileLog.DRIVE_CYCLE) {
      lineFollowing.displayLineSensors();
    }
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
    log.writeLogEcho("Robot", "Disabled", "");
    climb.enableCompressor(true);
    vision.setLedMode(1);

    wrist.stopWrist();
    elevator.stopElevator();
    climb.stopClimb();
    climb.enableVacuum(false);
    
    driveTrain.zeroGyroRotation(); 
    driveTrain.zeroLeftEncoder();
    driveTrain.zeroRightEncoder();
    leds.setColor(LedHandler.Color.OFF, false);
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();

    double pipeline = SmartDashboard.getNumber("Vision pipeline", 2.0);
    vision.setPipe(pipeline);

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString code to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons to
   * the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    log.writeLogEcho("Robot", "Autonomous mode init", "");
    beforeFirstEnable = false; // set variable that robot has been enabled
    m_autonomousCommand = new VisionSandstormSetup(); //m_chooser.getSelected();

    climb.enableCompressor(true);

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    climb.enableCompressor(true);
    vision.setLedMode(3);
    log.writeLogEcho("Robot", "Teleop mode init", "");
    beforeFirstEnable = false; // set variable that robot has been enabled
    
    driveTrain.zeroGyroRotation(); 
    driveTrain.zeroLeftEncoder();
    driveTrain.zeroRightEncoder();
    }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    vision.readCameraData(); // Don't think this is the issue of the loop time over run

    //SmartDashboard.putNumber("Target Distance", Robot.vision.distanceFromTarget()); // Distance isn't used for any segmentation
    SmartDashboard.putNumber("Target Quadrant", Robot.driveTrain.checkScoringQuadrant());
    //SmartDashboard.putBoolean("Vision Assistance Available", vision.areaFromCamera != 0); // This should work, need to test to see if there is a better metric to use
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    vision.setLedMode(3); // LEDs on during test for vision pipeline tuning
  }
}
