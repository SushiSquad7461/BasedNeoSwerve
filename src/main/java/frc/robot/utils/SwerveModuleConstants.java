package frc.robot.utils;

public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int angleMotorID;
    public final int canCoderID;
    public final double canCoderOffsetDegrees;

    public SwerveModuleConstants(int driveMotorID, int angleMotorID, int canCoderID, double canCoderOffsetDegrees) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.canCoderID = canCoderID;
        this.canCoderOffsetDegrees = canCoderOffsetDegrees;
    }
}
