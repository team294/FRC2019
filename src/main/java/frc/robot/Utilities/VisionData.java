package frc.robot.Utilities;

import frc.robot.Robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionData {
    public double horizOffset;    //  Horizontal angle error
    public double vertOffset;       // Vertical angle error
    
    public double xFromCamera,yFromCamera,areaFromCamera,ledMode;
    private NetworkTableEntry ledM;  // led mode 
    
    public NetworkTableEntry xValue,yValue,aValue;
    /**
     * Creates a VisionData object and connects to Limelight Camera
     */
    public VisionData() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    inst.startClientTeam(294);
    NetworkTable limelight = inst.getTable("limelight");
    
    ledM = limelight.getEntry("ledMode");
   
    xValue = limelight.getEntry("tx");
    yValue = limelight.getEntry("ty");
    aValue = limelight.getEntry("ta");

    // Aim error and angle error based on calibrated limelight cross-hair
    // aimXError = limelight.getEntry("cx0");  // aim error from CrossHair
    }

    public void readCameraData() {
        turnOnCamLeds();
        horizOffset = xValue.getDouble(0);
        vertOffset = yValue.getDouble(0);
        areaFromCamera = aValue.getDouble(0); 
        ledMode = ledM.getDouble(0);   
        // These functions are too taxing on CPU resources on the driver station
        //SmartDashboard.putNumber("Area", areaFromCamera);
        //SmartDashboard.putNumber("Angle to Crosshair", horizOffset);
        //SmartDashboard.putNumber("Distance", distanceFromTarget()); 
    }

   // Turn the LEDS on
    public void turnOnCamLeds() {
        ledM.setDouble(3);           
    }

    // Turn the LEDS off
    public void turnOffCamLeds() {
        ledM.setDouble(1);  
    }

      /**
     * 
     * @param modeNumber select a number from 0 to 3.
     * 0 = the LED Mode set in the current pipeline (find this at the ip address of the limelight).
     * 1 = off.
     * 2 = blink.
     * 3 = on.
     * If a number other than 0 to 3 is selected, turn the LEDs off.
     */
    public void setCameraMode(int modeNumber){
        if(modeNumber > 3 || modeNumber < 0){
            modeNumber = 1;
        }
        ledM.setDouble(modeNumber);
    }

    public void updateVisionLog() {
        Robot.log.writeLog("Vision", "Update Variables", "HorizOffset," + horizOffset + ",AreaFromCamera," + areaFromCamera);
    }
    
    /**
     *  questionable accuracy
    * @return the distance from the target in inches
    * 
    */
    // test comment
    public double distanceFromTarget (){
        double myDistance = 0.0;
        double cameraOffset = 12.0;
    
        // reference distance = 23.75 inches
        // reference area =  3.5 (the units that are used in limelight)
        //myDistance = 23.75 * Math.sqrt(areaFromCamera/3.5);
        myDistance = 23.75 * Math.sqrt(3.5/areaFromCamera) - cameraOffset;

        System.out.println("Distance from Target = " + myDistance);
        System.out.println("Area from Camera = " + areaFromCamera);
        return myDistance;
    }
}
