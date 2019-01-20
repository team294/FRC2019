package frc.robot.pathfinder;

import java.io.File;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;

import frc.robot.pathfinder.Trajectory.Segment;

/**
 * The main class of the Pathfinder Library. The Pathfinder Library is used for Motion Profile and Trajectory Generation.
 *
 * This class contains some of the most common methods you will use when doing Motion Profiling
 *
 * @author Jaci
 */
public class Pathfinder {

    /**
     * Convert degrees to radians. This is included here for static imports. In this library, all angle values are
     * given in radians
     * @param degrees the degrees input
     * @return the input in radians
     */
    public static double d2r(double degrees) {
        return Math.toRadians(degrees);
    }

    /**
     * Convert radians to degrees. This is included here for static imports.
     * @param radians the radians input
     * @return the input in degrees
     */
    public static double r2d(double radians) {
        return Math.toDegrees(radians);
    }

    /**
     * Bound an angle (in degrees) to -180 to 180 degrees.
	 * @param angle_degrees an input angle in degrees
	 * @return the bounded angle
     */
    public static double boundHalfDegrees(double angle_degrees) {
        while (angle_degrees >= 180.0) angle_degrees -= 360.0;
        while (angle_degrees < -180.0) angle_degrees += 360.0;
        return angle_degrees;
    }

    /**
     * Write the Trajectory to a CSV File
     * @param file          The file to write to
     * @param trajectory    The trajectory to write
     */
    public static void writeToCSV(File file, Trajectory trajectory) {
        // PathfinderJNI.trajectorySerializeCSV(trajectory.segments, file.getAbsolutePath());
    }

    /**
     * Read a Trajectory from a CSV File
     * @param fileName      The file to read from
     * @return              The trajectory that was read from file
     */
    public static Trajectory readFromCSV(String fileName) {
        String csvFile = "/Users/team/Documents/GitHub/FRC2019/PathWeaver/output/" + fileName;
        String line = "";
        String csvSplitBy = ",";
        int trajLength = 0;
        double dt, x, y, position, velocity, acceleration, jerk, heading;
        Segment seg;

        try (BufferedReader br = new BufferedReader(new FileReader(csvFile))) { 
            
            line = br.readLine();           // Read line with column headings and discard that line           
            while ((line = br.readLine()) != null) {
                trajLength++;
            }
            Segment[] segArray = new Segment[trajLength];
            int i = 0;
            line = br.readLine();           // Read line with column headings and discard that line           
            while ((line = br.readLine()) != null) {
                String[] traj = line.split(csvSplitBy);
                dt = Double.parseDouble(traj[0]);
                x = Double.parseDouble(traj[1]);
                y = Double.parseDouble(traj[2]);
                position = Double.parseDouble(traj[3]);
                velocity = Double.parseDouble(traj[4]);
                acceleration = Double.parseDouble(traj[5]);
                jerk = Double.parseDouble(traj[6]);
                heading = Double.parseDouble(traj[7]);
                seg = new Segment(dt, x, y, position, velocity, acceleration, jerk, heading);
                segArray[i] = seg;
                i++;
            }
            Trajectory traj = new Trajectory(segArray);
            return traj;
        } catch (IOException e) {
            e.printStackTrace();
            return null;
        }
        // return new Trajectory(PathfinderJNI.trajectoryDeserializeCSV(file.getAbsolutePath()));
    }

    /**
     * Thrown when a Trajectory could not be generated for an unknown reason.
     */
    public static class GenerationException extends Exception {
        public GenerationException(String message) {
            super(message);
        }
    }

}
