/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utilities.*;
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
  public static RearHatch rearHatch;
  public static VisionData vision;
  public static LineFollowing lineFollowing;
  public static Climb climb;
  public static OI oi;
  public static FileLog log;
  public static RobotPreferences robotPrefs;
  public static PowerDistributionPanel pdp;
  public static LedHandler leds;
  public static CANDeviceFinder canDeviceFinder;
  public static AutoSelection autoSelection;

  public static boolean beforeFirstEnable = true; // true before the first time the robot is enabled after loading code
  public static boolean startedAuto = false;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    // Enumerate the CANBus
    canDeviceFinder = new CANDeviceFinder();
    canDeviceFinder.enumerateCANBusToStdOut();

    // Create file log first, so any other class constructors can log data
    log = new FileLog("F0");

    // Read robot preference next, so any other class constructors can use preferences 
    robotPrefs = new RobotPreferences();
    robotPrefs.doExist();   // Sets up Robot Preferences if they do not exist : ie you just replaced RoboRio
    
    beforeFirstEnable = true; // set variable that robot has not been enabled

    // Create all subsystems and utilities
    if (robotPrefs.neoDrivetrain) {
      driveTrain = new NeoDriveTrain();
    } else {
      driveTrain = new CimDriveTrain();
    }
    
    shifter = new Shifter();
    elevator = new Elevator();
    wrist = new Wrist();
    cargo = new Cargo();
    hatch = new Hatch();
    rearHatch = new RearHatch();
    vision = new VisionData();
    vision.setLedMode(1);       // TODO Turn on (set to 3) if we are using vision

    lineFollowing = new LineFollowing();
    climb = new Climb();
    leds = new LedHandler();
    pdp = new PowerDistributionPanel();

    // pdp.clearStickyFaults();
    // m_chooser.setDefaultOption("Default Auto", new ExampleCommand());
    // chooser.addOption("My Auto", new MyAutoCommand());
    climb.enableCompressor(true);
    vision.setStreamMode(0);

    // Create auto selection utility
    autoSelection = new AutoSelection();
    
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

    vision.readCameraData();
    //SmartDashboard.putNumber("Target Distance", Robot.vision.distanceFromTarget()); // Distance isn't used for any segmentation
    SmartDashboard.putNumber("Target Quadrant", Robot.driveTrain.checkScoringQuadrant());
    //SmartDashboard.putBoolean("Vision Assistance Available", vision.areaFromCamera != 0); // This should work, need to test to see if there is a better metric to use

    if (!Robot.hatch.isHatchGrabbed()) Robot.oi.setXBoxRumble(1.0);
    else Robot.oi.setXBoxRumble(0);

    if (Robot.climb.isVacuumPresent()) Robot.leds.setColor(LedHandler.Color.BLUE, false); // solid BLUE when vacuum drawn
    else if (Robot.climb.getVacuumPressure(false) > 7.0) Robot.leds.setColor(LedHandler.Color.BLUE, true); // blinking BLUE when vacuum is starting to rise
    else if (Robot.vision.areaFromCamera > 0.1 && Robot.vision.areaFromCamera < 4.7) Robot.leds.setColor(LedHandler.Color.GREEN, false); // blinking GREEN when limelight target has specified area
    else if (Robot.cargo.getPhotoSwitch()) Robot.leds.setColor(LedHandler.Color.RED, false); // solid RED when cargo is present
    else if (Robot.hatch.isHatchGrabbed()) Robot.leds.setColor(LedHandler.Color.RED, true); // blinking RED when hatch piston is in extended position
    // else Robot.leds.setOff(); // if none of the above are true, turn OFF leds
    else Robot.leds.setColor(LedHandler.Color.OFF, false);    
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
    rearHatch.stopRearHatch();
    
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
    
    autoSelection.selectPath();
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
    // beforeFirstEnable = false; // set variable that robot has been enabled
    // elevator.setElevatorMotorPercentOutput(-0.2); // drive elevator down in case it isn't calibrated
    // m_autonomousCommand = new VisionSandstormSetup(); //m_chooser.getSelected();

    // climb.enableCompressor(true);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
    // if (autoSelection.autonomousCommand != null && !startedAuto) {
    //   autoSelection.autonomousCommand.start();
    //   Robot.log.writeLogEcho("AutoSelection", "Started Path", autoSelection.autonomousCommand.getName());
    //   startedAuto = true;
		// } else if (autoSelection.autonomousCommand == null && !startedAuto) {
    //   autoSelection.selectPath();
    // }
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    climb.enableCompressor(true);
    // vision.setLedMode(1);       // TODO Turn on (set to 3) if we are using vision
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
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    vision.setLedMode(3); // LEDs on during test for vision pipeline tuning
  }
}
