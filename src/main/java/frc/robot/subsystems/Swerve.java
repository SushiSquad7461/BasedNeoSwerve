package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.SwerveModule;

public class Swerve extends SubsystemBase {
  private final SwerveModule[] modules;

  private final SwerveDriveOdometry swerveOdometry;

  private final Pigeon2 gyro;

  public Swerve() {
    gyro = new Pigeon2(Constants.kSwerve.PIGEON2_ID);
    gyro.configFactoryDefault();
    zeroGyro();

    modules = new SwerveModule[] {
      new SwerveModule(0, Constants.kSwerve.MOD_0_Constants),
      new SwerveModule(1, Constants.kSwerve.MOD_1_Constants),
      new SwerveModule(2, Constants.kSwerve.MOD_2_Constants),
      new SwerveModule(3, Constants.kSwerve.MOD_3_Constants),
    };

    swerveOdometry = new SwerveDriveOdometry(Constants.kSwerve.KINEMATICS, getYaw());
  }

  /** 
   * This is called a command factory method, and these methods help reduce the
   * number of files in the command folder, increasing readability and reducing
   * boilerplate. */
  public Command drive(Joystick joystick, int xTranslationAxis, int yTranslationAxis, int rotationAxis, boolean isFieldRelative, boolean isOpenLoop) {
    return new RunCommand(() -> {
      // Grabbing input from joysticks.
      double xTranslation = joystick.getRawAxis(xTranslationAxis);
      double yTranslation = joystick.getRawAxis(yTranslationAxis);
      double rotation = joystick.getRawAxis(rotationAxis);

      // Adding deadzone.
      xTranslation = Math.abs(xTranslation) < Constants.kControls.AXIS_DEADZONE ? 0 : xTranslation;
      yTranslation = Math.abs(yTranslation) < Constants.kControls.AXIS_DEADZONE ? 0 : yTranslation;
      rotation = Math.abs(rotation) < Constants.kControls.AXIS_DEADZONE ? 0 : rotation;

      // Get desired module states.
      ChassisSpeeds chassisSpeeds = isFieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xTranslation, yTranslation, rotation, getYaw())
        : new ChassisSpeeds(xTranslation, yTranslation, rotation);

      SwerveModuleState[] states = Constants.kSwerve.KINEMATICS.toSwerveModuleStates(chassisSpeeds);

      setModuleStates(states, isOpenLoop);
    }, this);
  }

  /** To be used by auto. Use the drive method during teleop. */
  public void setModuleStates(SwerveModuleState[] states, boolean isOpenLoop) {
    // Makes sure the module states don't exceed the max speed.
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.kSwerve.MAX_VELOCITY_METERS_PER_SECOND);

    for (int i = 0; i < states.length; i++) {
      modules[i].setState(states[i], isOpenLoop);
    }
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState currentStates[] = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      currentStates[i] = modules[i].getState();
    }

    return currentStates;
  }

  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(gyro.getYaw());
  }

  public Command getZeroGyroCommand() {
    return new InstantCommand(this::zeroGyro);
  }

  private void zeroGyro() {
    gyro.setYaw(0);
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(pose, getYaw());
  }

  @Override
  public void periodic() {
    swerveOdometry.update(getYaw(), getStates());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    for (SwerveModule module : modules) {
      builder.addStringProperty(
        String.format("Module %d",
        module.moduleNumber),
        () -> {
          SwerveModuleState state = module.getState();
          return String.format("%.2fm/s %.0frad", state.speedMetersPerSecond, state.angle.getRadians());
        },
        null);
    }
  }
}
