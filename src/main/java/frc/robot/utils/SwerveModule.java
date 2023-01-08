package frc.robot.utils;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class SwerveModule {
  public final int moduleNumber;

  private final CANSparkMax driveMotor;
  private final RelativeEncoder driveEncoder;
  private final SparkMaxPIDController drivePID;
  private final SimpleMotorFeedforward driveFeedforward;

  private final CANSparkMax angleMotor;
  private final RelativeEncoder angleEncoder;
  private final SparkMaxPIDController anglePID;
  
  private final CANCoder canCoder;
  private final double canCoderOffsetDegrees;

  private double lastAngle;

  public SwerveModule(int moduleNumber, SwerveModuleConstants constants) {
    this.moduleNumber = moduleNumber;
    
    driveMotor = new CANSparkMax(constants.driveMotorID, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    drivePID = driveMotor.getPIDController();
    driveFeedforward = new SimpleMotorFeedforward(Constants.kSwerve.DRIVE_KS, Constants.kSwerve.DRIVE_KV, Constants.kSwerve.DRIVE_KA);

    angleMotor = new CANSparkMax(constants.angleMotorID, MotorType.kBrushless);
    angleEncoder = angleMotor.getEncoder();
    anglePID = angleMotor.getPIDController();

    canCoder = new CANCoder(constants.canCoderID);
    canCoderOffsetDegrees = constants.canCoderOffsetDegrees;

    configureDevices();
    lastAngle = getState().angle.getRadians();
  }

  public void setState(SwerveModuleState state, boolean isOpenLoop) {
    // Prevents angle motor from turning further than it needs to. 
    // E.G. rotating from 10 to 270 degrees CW vs CCW.
    state = SwerveModuleState.optimize(state, getState().angle);

    if (isOpenLoop) {
      double speed = state.speedMetersPerSecond / Constants.kSwerve.MAX_VELOCITY_METERS_PER_SECOND;
      drivePID.setReference(speed, CANSparkMax.ControlType.kDutyCycle);
    } else {
      drivePID.setReference(state.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity, 0, driveFeedforward.calculate(state.speedMetersPerSecond));
    }

    double angle = Math.abs(state.speedMetersPerSecond) <= Constants.kSwerve.MAX_VELOCITY_METERS_PER_SECOND * 0.01
      ? lastAngle
      : state.angle.getRadians();

    anglePID.setReference(angle, CANSparkMax.ControlType.kPosition);
    lastAngle = angle;
  }

  public SwerveModuleState getState() {
    double velocity = driveEncoder.getVelocity();
    Rotation2d rot = new Rotation2d(angleEncoder.getPosition());
    return new SwerveModuleState(velocity, rot);
  }

  public double getCanCoder() {
    return canCoder.getAbsolutePosition();
  }

  public Rotation2d getAngle() {
    return new Rotation2d(angleEncoder.getPosition());
  }

  public SwerveModulePosition getPosition() {
    double distance = driveEncoder.getPosition();
    Rotation2d rot = new Rotation2d(angleEncoder.getPosition());
    return new SwerveModulePosition(distance, rot);
  }

  private void configureDevices() {
    // Drive motor configuration.
    driveMotor.restoreFactoryDefaults();
    driveMotor.setInverted(Constants.kSwerve.DRIVE_MOTOR_INVERSION);
    driveMotor.setIdleMode(Constants.kSwerve.DRIVE_IDLE_MODE);
    driveMotor.setOpenLoopRampRate(Constants.kSwerve.OPEN_LOOP_RAMP);
    driveMotor.setClosedLoopRampRate(Constants.kSwerve.CLOSED_LOOP_RAMP);
    driveMotor.setSmartCurrentLimit(Constants.kSwerve.DRIVE_CURRENT_LIMIT);
 
    drivePID.setP(Constants.kSwerve.DRIVE_KP);
    drivePID.setI(Constants.kSwerve.DRIVE_KI);
    drivePID.setD(Constants.kSwerve.DRIVE_KD);
    drivePID.setFF(Constants.kSwerve.DRIVE_KF);
 
    driveEncoder.setPositionConversionFactor(Constants.kSwerve.DRIVE_ROTATIONS_TO_METERS);
    driveEncoder.setVelocityConversionFactor(Constants.kSwerve.DRIVE_RPM_TO_METERS_PER_SECOND);
    driveEncoder.setPosition(0);

    // Angle motor configuration.
    angleMotor.restoreFactoryDefaults();
    angleMotor.setInverted(Constants.kSwerve.ANGLE_MOTOR_INVERSION);
    angleMotor.setIdleMode(Constants.kSwerve.ANGLE_IDLE_MODE);
    angleMotor.setSmartCurrentLimit(Constants.kSwerve.ANGLE_CURRENT_LIMIT);

    anglePID.setP(Constants.kSwerve.ANGLE_KP);
    anglePID.setI(Constants.kSwerve.ANGLE_KI);
    anglePID.setD(Constants.kSwerve.ANGLE_KD);
    anglePID.setFF(Constants.kSwerve.ANGLE_KF);

    anglePID.setPositionPIDWrappingEnabled(true);
    anglePID.setPositionPIDWrappingMaxInput(2 * Math.PI);
    anglePID.setPositionPIDWrappingMinInput(0);

    angleEncoder.setPositionConversionFactor(Constants.kSwerve.ANGLE_ROTATIONS_TO_RADIANS);
    angleEncoder.setVelocityConversionFactor(Constants.kSwerve.ANGLE_RPM_TO_RADIANS_PER_SECOND);
    angleEncoder.setPosition(Units.degreesToRadians(canCoder.getAbsolutePosition() - canCoderOffsetDegrees));

    // CanCoder configuration.
    CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
    canCoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    canCoderConfiguration.sensorDirection = Constants.kSwerve.CANCODER_INVERSION;
    canCoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    canCoderConfiguration.sensorTimeBase = SensorTimeBase.PerSecond;
    
    canCoder.configFactoryDefault();
    canCoder.configAllSettings(canCoderConfiguration);
  }
}
