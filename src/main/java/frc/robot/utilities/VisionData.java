package frc.robot.utilities;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionData {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable limelight = inst.getTable("limelight");

    public double horizOffset;    //  Horizontal angle error
    public double vertOffset;       // Vertical angle error
    
    public double areaFromCamera,ledMode;
    private NetworkTableEntry ledM, pipeline;  // led mode 
    
    public NetworkTableEntry xValue,yValue,aValue;
    /**
     * Creates a VisionData object and connects to Limelight Camera
     */
    public VisionData() {

        inst.startClientTeam(294);

        ledM = limelight.getEntry("ledMode");
        pipeline = limelight.getEntry("pipeline");
    
        xValue = limelight.getEntry("tx");
        yValue = limelight.getEntry("ty");
        aValue = limelight.getEntry("ta");

    // Aim error and angle error based on calibrated limelight cross-hair
    // aimXError = limelight.getEntry("cx0");  // aim error from CrossHair
    }

    public void readCameraData() {       
        horizOffset = xValue.getDouble(0);
        vertOffset = yValue.getDouble(0);
        areaFromCamera = aValue.getDouble(0); 
        ledMode = ledM.getDouble(0);
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
     * Returns the true pipeline being used
     * @return
     */
    public double getPipeline() {
        return limelight.getEntry("getpipe").getDouble(0);
    }

    /**
     * Sets the pipeline number to use
     * @param pipeNum Pipeline to change to (see limelight web dashboard for details)
     */
    public void setPipe(double pipeNum) {
        pipeline.setDouble(pipeNum);
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
    
    /**
     *  questionable accuracy
    * @return the distance from the target in inches
    * 
    */
    public double distanceFromTarget (){
        double myDistance = 0.0;
        double cameraOffset = 12.0;
    
        // reference distance = 23.75 inches
        // reference area =  3.5 (the units that are used in limelight)
        //myDistance = 23.75 * Math.sqrt(areaFromCamera/3.5);
        myDistance = 23.75 * Math.sqrt(3.5/areaFromCamera) - cameraOffset;

        //myDistance = (targetHeight - camMountHeight) / Math.tan(camMountAngle + vertOffset);

        return myDistance;
    }

    // TODO: Redo entire distance formula to be based on height (horiz offset) which also means standardizing crosshair y value
}
