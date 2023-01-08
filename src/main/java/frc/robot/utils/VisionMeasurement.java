package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;

public class VisionMeasurement {
    public final Pose2d pose;
    public final double timestampSeconds;
    public final double ambiguity;
    
    public VisionMeasurement(Pose2d pose, double timestampSeconds, double ambiguity) {
        this.pose = pose;
        this.timestampSeconds = timestampSeconds;
        this.ambiguity = ambiguity;
    }
}
