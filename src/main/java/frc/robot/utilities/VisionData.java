package frc.robot.utilities;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionData {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable limelight = inst.getTable("limelight");

    public double horizOffset;    //  Horizontal angle error
    public double vertOffset;       // Vertical angle error
    public double distance;         // Distance to target in inches, calculated from area
    public double distanceUsingCorners;   // Distance to target in inches, calculated from contour corners
    public double skew;         // Skew angle of target, -45 to +45 degrees
    public boolean valid;       // Do we have a valid target?
    public boolean vtemp;
    
    public double areaFromCamera,ledMode;

    public double[] cornX, cornY;

    private NetworkTableEntry ledM, pipeline, camMode, stream, snapshot;
    private NetworkTableEntry nteX, nteY, nteA, nteS;
    private NetworkTableEntry nteV, nteCornX, nteCornY;

    private double[] defaultArray = new double[0];          // Default value for returning arrays for networktables

    /**
     * Creates a VisionData object and connects to Limelight Camera
     */
    public VisionData() {

        inst.startClientTeam(294);

        ledM = limelight.getEntry("ledMode");
        camMode = limelight.getEntry("camMode");
        pipeline = limelight.getEntry("pipeline");
        stream = limelight.getEntry("stream");
        snapshot = limelight.getEntry("snapshot");
    
        nteX = limelight.getEntry("tx");
        nteY = limelight.getEntry("ty");
        nteA = limelight.getEntry("ta");
        nteS = limelight.getEntry("ts");

        nteV = limelight.getEntry("tv");
        nteCornX = limelight.getEntry("tcornx");
        nteCornY = limelight.getEntry("tcorny");

        SmartDashboard.putNumber("Vision pipeline", 2.0);
    // Aim error and angle error based on calibrated limelight cross-hair
    // aimXError = limelight.getEntry("cx0");  // aim error from CrossHair
    }

    public void readCameraData() {
        valid = (nteV.getDouble(0) == 1);
        horizOffset = nteX.getDouble(0);
        vertOffset = nteY.getDouble(0);
        areaFromCamera = nteA.getDouble(0); 
        ledMode = ledM.getDouble(0);
        distance = 76.48 / Math.sqrt(areaFromCamera); // Distance in inches, center of limelight to center of target
        skew = nteS.getDouble(0);
        skew = (skew<-45) ? skew+90 : skew;  // convert skew from (-90, 0) to (-45, 45)

        cornX = nteCornX.getDoubleArray(defaultArray);
        cornY = nteCornY.getDoubleArray(defaultArray);

        // Re-check valid, to see if it has changed
        valid = valid && (nteV.getDouble(0) == 1);



        SmartDashboard.putBoolean("Vision valid", valid);
        SmartDashboard.putNumber("Vision X", horizOffset);
        SmartDashboard.putNumber("Vision Y", vertOffset);
        SmartDashboard.putNumber("Vision Area", areaFromCamera);
        SmartDashboard.putNumber("Vision Distance", distance);
        SmartDashboard.putNumber("Vision Skew", skew);
    }

    private boolean calcDistanceUsingCorners() {
        if (!valid || cornX.length != 8 || cornY.length != 8) {
            distanceUsingCorners = 0;
            SmartDashboard.putNumber("Vision DistUC rawDX", cornX[7]-cornX[0]);
            return false;
        }

        SmartDashboard.putNumber("Vision DistUC rawDX", cornX[7]-cornX[0]);

        return true;
    }

    /*
   // Turn the LEDS on
    public void turnOnCamLeds() {
        ledM.setDouble(3);           
    }

    // Turn the LEDS off
    public void turnOffCamLeds() {
        ledM.setDouble(1);  
    }
    */

    /**
     * Returns the true pipeline being used
     * @return
     */
    public double getPipeline() {
        return limelight.getEntry("getpipe").getDouble(0);
    }

    /**
     * Sets the pipeline number to use
     * @param pipeNum Pipeline to change to (see limelight web dashboard for details).   (0 = vision, 2 = driver feed)
     */
    public void setPipe(double pipeNum) {
        pipeline.setDouble(pipeNum);
    }

    /**
     * Sets the streaming mode of the cameras
     * @param mode 0 = side-by-side
     * 1 = driver camera lower right corner of vision
     * 2 = vision camera lower right corner of driver camera
     */
    public void setStreamMode(double mode) {
        stream.setDouble(mode);
    }

    /**
     * Enables or disables snapshotting during the match
     * @param mode 0 = no snapshots
     * 1 = two snapshots per second
     */
    public void setSnapshot(double mode) {
        snapshot.setDouble(mode);
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
    public void setLedMode(int modeNumber) {
        if (modeNumber > 3 || modeNumber < 0) modeNumber = 1;
        ledM.setDouble(modeNumber);
    }

    /**
     * Sets the mode of the camera for use as driver cam or vision processing
     * @param mode 0 = vision; 1 = driver camera
     */
    public void setCamMode(int mode) {
        camMode.setDouble(mode);
    }
    
}
