package frc.robot.pathfinder.followers;

import frc.robot.pathfinder.Trajectory;

/**
 * The DistanceFollower is an object designed to follow a trajectory based on distance covered input. This class can be used
 * for Tank or Swerve drive implementations.
 *
 * @author Jaci
 */
public class DistanceFollower {

    Trajectory trajectory;              // Trajectory to follow
    double kp, ki, kd, kv, ka;          // PIDVA constants for follower

    double dt;                          // time delta between points in trajectory array
    double last_error;                  // distance error from last calculation (for derivative calc)

    int segIndex;                       // index of current segment.  In real time, we are between segIndex and segIndex+1
    double segOffset;                   // fractional offset of current time between segIndex and segIndex+1  (value should be between 0 and 1)
    Trajectory.Segment seg;             // current segment (values may be interpolated from within trajectory array)

    long startTimeMillis, calcTimeMillis;  // Start time for trajectory following, current time in trajectory


    /**
     * Create a new distance follower that follows a trajectory
     * @param traj Trajectory to follow
     */
    public DistanceFollower(Trajectory traj) {
        setTrajectory(traj);
    }

    public DistanceFollower() { }

    /**
     * Set a new trajectory to follow, and reset the cumulative errors and segment counts
     * @param traj a previously generated trajectory
     */
    public void setTrajectory(Trajectory traj) {
        trajectory = traj;
        dt = trajectory.segments[0].dt;
        reset();
    }

    /**
     * Reset the follower to start (or start again).
     */
    public void reset() {
        last_error = 0.0; 
        segIndex = 0;
        segOffset = 0.0;
        startTimeMillis = System.currentTimeMillis();
        calcTimeMillis = startTimeMillis;
        seg = trajectory.segments[0].copy();
    }

    /**
     * Configure the PID/VA Variables for the Follower
     * @param kp The proportional term. This is usually quite high (0.8 - 1.0 are common values)
     * @param ki The integral term. Currently unused.
     * @param kd The derivative term. Adjust this if you are unhappy with the tracking of the follower. 0.0 is the default
     * @param kv The velocity ratio. This should be 1 over your maximum velocity @ 100% throttle.
     *           This converts m/s given by the algorithm to a scale of -1..1 to be used by your
     *           motor controllers
     * @param ka The acceleration term. Adjust this if you want to reach higher or lower speeds faster. 0.0 is the default
     */
    public void configurePIDVA(double kp, double ki, double kd, double kv, double ka) {
        this.kp = kp; this.ki = ki; this.kd = kd;
        this.kv = kv; this.ka = ka;
    }

    /**
     * Calculate the desired output for the motors, based on the distance the robot has covered.
     * This does not account for heading of the robot. To account for heading, add some extra terms in your control
     * loop for realignment based on gyroscope input and the desired heading given by this object.
     * @param distance_covered  The distance covered in inches
     * @return                  The desired output for your motor controller
     */
    public double calculate(double distance_covered) {
        Trajectory.Segment seg_next;        // next segment after current segment (for interpolation)
        double error, calculated_value;

        // Calcluate current position in trajectory based on real time
        calcTimeMillis = System.currentTimeMillis();
        segOffset = ((double)(calcTimeMillis - startTimeMillis))/1000.0/dt;
        segIndex = (int) Math.floor(segOffset);     // closest segment index that we just went past
        segOffset = segOffset - segIndex;      // fractional offset to next segment

        // Check if we are past the end of the trajectory
        if (segIndex >= trajectory.length()-1) {
            // Copy the last segment in the array to seg, in case someone calls getSegment()
            trajectory.segments[trajectory.length()-1].copyTo(seg);
            return 0;
        }

        trajectory.get(segIndex).copyTo(seg);     // Get the segment we just passed
        seg_next = trajectory.get(segIndex+1);    // Get the next segment

        // interpolate the position and velocity that we should be at right now
        seg.position += (seg_next.position - seg.position)*segOffset;
        seg.velocity += (seg_next.velocity - seg.velocity)*segOffset;

        error = seg.position - distance_covered;
        calculated_value =
                kp * error +                                    // Proportional
                kd * ((error - last_error) / dt) +              // Derivative
                (kv * seg.velocity + ka * seg.acceleration);    // V and A Terms
        last_error = error;

        return calculated_value;
    }

    /**
     * @return the desired heading of the current point in the trajectory
     */
    public double getHeading() {
        return seg.heading;
    }

    /**
     * @return the current segment object being operated on, including interpolation
     */
    public Trajectory.Segment getSegment() {
        return seg;
    }

    /**
     * @return the current segment index being operated on.  The current segment is 
     * actually an interpolation between this index and the next index.
     */
    public int getSegmentIndex() {
        return segIndex;
    }

    /**
     * @return the interpolation factor between getSegmentIndex() and the next segment
     */
    public double getSegmentOffset() {
        return segOffset;
    }

    /**
     * @return the current time when getSegment() was last calculated (in the same
     * format as System.currentTimeMillis())
     */
    public long getCalcTimeMillis() {
        return calcTimeMillis;
    }

    /**
     * @return the time when getSegment() was last calculated, as an offset in milliseconds
     * since this DistanceFollower was last reset.
     */
    public long getCalcDeltaTimeMillis() {
        return calcTimeMillis-startTimeMillis;
    }

    /**
     * @return whether we have finished tracking this trajectory or not.
     */
    public boolean isFinished() {
        return segIndex >= trajectory.length();
    }

}
