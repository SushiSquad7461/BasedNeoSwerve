package frc.robot.subsystems;

import java.util.List;
import java.util.function.Supplier;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utils.SwerveModule;
import frc.robot.utils.VisionMeasurement;

public class Swerve extends SubsystemBase {
  private final RobotContainer container;

  private final SwerveModule[] modules;

  private final SwerveDrivePoseEstimator swerveOdometry;
  private final Field2d field;

  private final AHRS gyro;

  public Swerve(RobotContainer container) {
    this.container = container;

    gyro = new AHRS();
    zeroGyro();

    modules = new SwerveModule[] {
      new SwerveModule(0, Constants.kSwerve.MOD_0_Constants),
      new SwerveModule(1, Constants.kSwerve.MOD_1_Constants),
      new SwerveModule(2, Constants.kSwerve.MOD_2_Constants),
      new SwerveModule(3, Constants.kSwerve.MOD_3_Constants),
    };

    swerveOdometry = new SwerveDrivePoseEstimator(
      Constants.kSwerve.KINEMATICS,
      getYaw(),
      getPositions(),
      new Pose2d(),
      Constants.kSwerve.STATE_STANDARD_DEVIATION,
      Constants.kSwerve.VISION_STANDARD_DEVIATION
    );
    field = new Field2d();
  }

  /** 
   * This is called a command factory method, and these methods help reduce the
   * number of files in the command folder, increasing readability and reducing
   * boilerplate. 
   * 
   * The supplier is a function that returns an array of [fowardback, leftright, theta].
   */
  public Command drive(Supplier<double[]> transformSupplier, boolean isFieldRelative, boolean isOpenLoop) {
    return run(() -> {
      // Grabbing input from suppliers.
      double[] transform = transformSupplier.get();
      SmartDashboard.putNumberArray("movement", transform);
      double forwardBack = transform[0];
      double leftRight = transform[1];
      double rotation = transform[2];

      // Adding deadzone.
      forwardBack = Math.abs(forwardBack) < Constants.kControls.AXIS_DEADZONE ? 0 : forwardBack;
      leftRight = Math.abs(leftRight) < Constants.kControls.AXIS_DEADZONE ? 0 : leftRight;
      rotation = Math.abs(rotation) < Constants.kControls.AXIS_DEADZONE ? 0 : rotation;

      // Limiting to one.
      forwardBack = MathUtil.clamp(forwardBack, -1, 1);
      leftRight = MathUtil.clamp(leftRight, -1, 1);
      rotation = MathUtil.clamp(rotation, -1, 1);

      // Get desired module states.
      ChassisSpeeds chassisSpeeds = isFieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(forwardBack, leftRight, rotation, getYaw())
        : new ChassisSpeeds(forwardBack, leftRight, rotation);

      SwerveModuleState[] states = Constants.kSwerve.KINEMATICS.toSwerveModuleStates(chassisSpeeds);

      setModuleStates(states, isOpenLoop);
    }).withName("SwerveDriveCommand");
  }

  /** To be used by auto. Use the drive method during teleop. */
  public void setModuleStates(SwerveModuleState[] states) {
    setModuleStates(states, false);
  }

  private void setModuleStates(SwerveModuleState[] states, boolean isOpenLoop) {
    // Makes sure the module states don't exceed the max speed.
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.kSwerve.MAX_VELOCITY_METERS_PER_SECOND);

    for (int i = 0; i < modules.length; i++) {
      modules[i].setState(states[modules[i].moduleNumber], isOpenLoop);
    }
  }

  public Command maintainDistanceFromAprilTag(double distanceMeters) {
    PIDController distancePID = new PIDController(0.3, 0, 0);
    PIDController thetaPID = new PIDController(0.03, 0, 0);

    distancePID.setSetpoint(distanceMeters);
    thetaPID.setSetpoint(0);

    Supplier<double[]> supplier = () -> {
      double[] transform = new double[3];
      
      PhotonTrackedTarget target = container.vision.getBestTarget();
      if (target == null) {
        return transform;
      }

      Translation3d translation = target.getBestCameraToTarget().getTranslation();
      Translation2d translation2d = new Translation2d(translation.getX(), translation.getY());

      double distance = distancePID.calculate(Math.max(distanceMeters, translation2d.getDistance(new Translation2d())));
      Translation2d finalTranslation = new Translation2d(
        distance, 
        Rotation2d.fromDegrees(-target.getYaw())
          .plus(Constants.kVision.CAMERA_ANGLE_DEGREES.toRotation2d().unaryMinus()));

      transform[0] = finalTranslation.getX();
      transform[1] = finalTranslation.getY();
      transform[2] = thetaPID.calculate(target.getYaw());

      return transform;
    };

    return drive(supplier, false, false)
      .andThen(() -> { distancePID.close(); thetaPID.close(); });
  }

  public Command odometryDrive(Supplier<double[]> transformSupplier) {
    PIDController yPID = new PIDController(0.3, 0, 0);
    PIDController xPID = new PIDController(0.3, 0, 0);
    PIDController thetaPID = new PIDController(0.03, 0, 0);

    Supplier<double[]> supplier = () -> {
      double[] transform = transformSupplier.get();

      yPID.setSetpoint(yPID.getSetpoint() + transform[0] * Constants.kSwerve.MAX_VELOCITY_METERS_PER_SECOND * 0.02);
      xPID.setSetpoint(xPID.getSetpoint() + transform[1] * Constants.kSwerve.MAX_VELOCITY_METERS_PER_SECOND * 0.02);
      thetaPID.setSetpoint(thetaPID.getSetpoint() + transform[2] * Constants.kSwerve.MAX_VELOCITY_METERS_PER_SECOND * 0.02);

      return new double[] {
        yPID.calculate(swerveOdometry.getEstimatedPosition().getY()),
        xPID.calculate(swerveOdometry.getEstimatedPosition().getX()),
        thetaPID.calculate(swerveOdometry.getEstimatedPosition().getRotation().getRadians())
      };
    };

    return drive(supplier, false, false)
      .andThen(() -> { yPID.close(); xPID.close(); thetaPID.close(); });
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState currentStates[] = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      currentStates[i] = modules[i].getState();
    }

    return currentStates;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition currentStates[] = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      currentStates[i] = modules[i].getPosition();
    }

    return currentStates;
  }

  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  public Command zeroGyroCommand() {
    return runOnce(this::zeroGyro).withName("ZeroGyroCommand");
  }

  private void zeroGyro() {
    gyro.zeroYaw();
  }

  public Pose2d getPose() {
    return swerveOdometry.getEstimatedPosition();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
  }

  public void resetSensors(Pose2d pose) {
    zeroGyro();
    gyro.setAngleAdjustment(pose.getRotation().unaryMinus().getDegrees());
    swerveOdometry.resetPosition(pose.getRotation(), getPositions(), pose);
  }

  @Override
  public void periodic() {
    swerveOdometry.update(getYaw(), getPositions());

    // Loop through all measurements and add it to pose estimator
    List<VisionMeasurement> measurements = container.vision.getMeasurements();
    if (measurements != null) {
      for (VisionMeasurement measurement : measurements) {
        // Skip measurement if it's more than a meter away
        if (measurement.pose.getTranslation().getDistance(swerveOdometry.getEstimatedPosition().getTranslation()) > 1.0) {
          continue;
        }
  
        swerveOdometry.addVisionMeasurement(
          measurement.pose,
          measurement.timestampSeconds,
          Constants.kSwerve.VISION_STANDARD_DEVIATION);//.times(measurement.ambiguity + 0.9)); 
      }
    }

    field.setRobotPose(swerveOdometry.getEstimatedPosition());
    SmartDashboard.putData(field);
    SmartDashboard.putNumber("angle", getYaw().getRadians());
    SmartDashboard.putNumber("angle odo", swerveOdometry.getEstimatedPosition().getRotation().getRadians());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    for (SwerveModule module : modules) {
      builder.addStringProperty(
        String.format("Module %d", module.moduleNumber),
        () -> {
          SwerveModuleState state = module.getState();
          return String.format("%6.2fm/s %6.3fdeg", state.speedMetersPerSecond, state.angle.getDegrees());
        },
        null);

        builder.addDoubleProperty(
          String.format("Cancoder %d", module.moduleNumber),
          () -> module.getCanCoder(),
          null);

          
        builder.addDoubleProperty(
          String.format("Angle %d", module.moduleNumber),
          () -> module.getAngle().getDegrees(),
          null);
    }
  }
}
