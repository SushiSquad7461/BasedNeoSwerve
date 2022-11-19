package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utils.VisionMeasurement;

public class Vision extends SubsystemBase {
  public final PhotonCamera camera;

  private ArrayList<VisionMeasurement> measurements = new ArrayList<>();
  private PhotonTrackedTarget bestTarget;
  private VisionMeasurement bestMeasurement;

  public Vision() {
    if (Robot.isReal()) {
      camera = new PhotonCamera("photonvision");
    } else {
      NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
      ntInstance.stopServer();
      ntInstance.setServer("localhost");
      ntInstance.startClient3("localhost");
      camera = new PhotonCamera(ntInstance, "Integrated_Webcam");
    }
  }

  public List<VisionMeasurement> getMeasurements() {
    return measurements;
  }

  public PhotonTrackedTarget getBestTarget() {
    return bestTarget;
  }

  public VisionMeasurement getBestMeasurement() {
    return bestMeasurement;
  }
  
  @Override
  public void periodic() {
    PhotonPipelineResult res = camera.getLatestResult();
    
    if (!res.hasTargets()) {
      return;
    }

    bestTarget = res.getBestTarget();
    Pose3d bestPose = getRobotPoseFromTarget(bestTarget);
    if (bestPose == null) {
      bestMeasurement = null;
    } else {
      bestMeasurement = new VisionMeasurement(
        new Pose2d(
          bestPose.getX(),
          bestPose.getY(),
          new Rotation2d(bestPose.getRotation().getZ())), 
        res.getLatencyMillis(),
        bestTarget.getPoseAmbiguity()); 
    }

    measurements.clear();
    for (PhotonTrackedTarget target : res.targets) {
      if (target.getPoseAmbiguity() > 0.2) {
        continue;
      }

      Pose3d estRobotPose = getRobotPoseFromTarget(target);
      if (estRobotPose == null) {
        continue;
      }

      measurements.add(new VisionMeasurement(
        new Pose2d(
          estRobotPose.getX(),
          estRobotPose.getY(),
          new Rotation2d(estRobotPose.getRotation().getZ())), 
        res.getLatencyMillis(),
        bestTarget.getPoseAmbiguity())); 
    }
  }

  public static Pose3d getRobotPoseFromTarget(PhotonTrackedTarget target) {
    Transform3d cameraToTarget = target.getBestCameraToTarget();
    Transform3d targetToCamera = cameraToTarget.inverse();

    Optional<Pose3d> feducialPos = Constants.kVision.APRIL_TAG_FIELD_LAYOUT
      .getTagPose(target.getFiducialId());

    if (feducialPos.isEmpty()) {
        return null;
    }

    Pose3d estCameraPose = feducialPos.get().transformBy(targetToCamera);
    Pose3d estRobotPos = estCameraPose.transformBy(Constants.kVision.CAMERA_TO_ROBOT_METERS_DEGREES);

    return estRobotPos;
  }
}
