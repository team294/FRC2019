/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Utilities;

import frc.robot.Robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Creates a VisionData object and connects to data from Limelight
 */

public class VisionData {
    public double aim_error;
    public NetworkTableEntry ledMode;

    public NetworkTableEntry XfromCamera,YfromCamera,AfromCamera;

    public VisionData() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    inst.startClientTeam(294);
    NetworkTable limelight = inst.getTable("Limelight");
    
    ledMode = limelight.getEntry("ledMode");
  
    turnOnCamLeds();
    XfromCamera = limelight.getEntry("tx");
    YfromCamera = limelight.getEntry("ty");
    AfromCamera = limelight.getEntry("ta");

    // Aim error and distance error based on calibrated limelight cross-hair
    aim_error = XfromCamera.getDouble(-1);
                
    }

// Turn the LEDS on
    public void turnOnCamLeds() {
        ledMode.setDouble(0);
    }

    // Turn the LEDS off
    public void turnOffCamLeds() {
        ledMode.setDouble(1);
    }

}
