package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public final Joystick driver;

  public final Swerve swerve;

  public final Vision vision;

  public RobotContainer() {
    driver = new Joystick(Constants.kControls.DRIVE_JOYSTICK_ID);

    swerve = new Swerve(this);

    vision = new Vision();

    // Configure button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    swerve.setDefaultCommand(swerve.drive(
      () -> -driver.getRawAxis(Constants.kControls.TRANSLATION_Y_AXIS),
      () -> -driver.getRawAxis(Constants.kControls.TRANSLATION_X_AXIS), 
      () -> -driver.getRawAxis(Constants.kControls.ROTATION_AXIS),
      true,
      false
    ));

    new JoystickButton(driver, Constants.kControls.GYRO_RESET_BUTTON)
      .onTrue(swerve.zeroGyroCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    PathPlannerTrajectory traj = PathPlanner.loadPath(
      "Test", 
      Constants.kAuto.MAX_VELOCITY_METERS_PER_SECOND, 
      Constants.kAuto.MAX_ACCEL_METERS_PER_SECOND_SQUARED);

    swerve.resetOdometry(traj.getInitialHolonomicPose());

    return new PPSwerveControllerCommand(
      traj,
      swerve::getPose,
      Constants.kSwerve.KINEMATICS,
      new PIDController(Constants.kAuto.X_CONTROLLER_KP, 0, 0),
      new PIDController(Constants.kAuto.Y_CONTROLLER_KP, 0, 0),
      new PIDController(Constants.kAuto.THETA_CONTROLLER_KP, 0, 0),
      swerve::setModuleStates,
      this.swerve);
  }
}
