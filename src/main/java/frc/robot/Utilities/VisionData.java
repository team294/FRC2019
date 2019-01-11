package frc.robot.Utilities;

import frc.robot.Robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Creates a VisionData object and connects to data from Limelight
 */

public class VisionData {
    public double aimError;    //  Horizontal angle error
    
    public double xFromCamera,yFromCamera,areaFromCamera,ledMode;
    public NetworkTableEntry ledM, aError;

    private NetworkTableEntry xValue,yValue,aValue;
    /**
     * Creates a VisionData object and connects to Limelight Camera
     */
    public VisionData() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    inst.startClientTeam(294);
    NetworkTable limelight = inst.getTable("Limelight");
    
    ledM = limelight.getEntry("ledMode");
   
    xValue = limelight.getEntry("tx");
    yValue = limelight.getEntry("ty");
    aValue = limelight.getEntry("ta");

    // Aim error and distance error based on calibrated limelight cross-hair
    aError = limelight.getEntry("cx0");
    }

    public void readCameraData() {
        turnOnCamLeds();
        xFromCamera = xValue.getDouble(0);
        yFromCamera = yValue.getDouble(0);
        areaFromCamera = aValue.getDouble(0); 
        aimError = aError.getDouble(0);
        ledMode = ledM.getDouble(0);   
        SmartDashboard.putNumber("Area", areaFromCamera);
        SmartDashboard.putNumber("Angle to Crosshair 1", aimError);
    }

   // Turn the LEDS on
    public  void turnOnCamLeds() {
        ledM.setDouble(3);   // This sets the field, but not contolling leds
        System.out.println("leds?");   
    }

    // Turn the LEDS off
    public void turnOffCamLeds() {
        ledM.setNumber(1);      //??
        //ledM.setDouble(1);// This sets the field, but not contolling leds??
        //NetworkTableInstance.getDefault().getTable(“LimeLight”).getEntry("ledMode").setNumber(1)
        System.out.println("leds OFF?"); 
    }

}
