package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;

public class VisionMeasurement {
    public final Pose2d pose;
    public final double latencyMillis;
    public final double ambiguity;
    
    public VisionMeasurement(Pose2d pose, double latencyMillis, double ambiguity) {
        this.pose = pose;
        this.latencyMillis = latencyMillis;
        this.ambiguity = ambiguity;
    }
}
