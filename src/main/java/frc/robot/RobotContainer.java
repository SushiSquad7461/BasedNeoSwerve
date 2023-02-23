package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public final Joystick driver;

  public final Swerve swerve;

  public final AutoCommands auto;

  public RobotContainer() {
    driver = new Joystick(Constants.kControls.DRIVE_JOYSTICK_ID);

    swerve = new Swerve();

    auto = new AutoCommands(swerve);

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
      () -> -Constants.kControls.X_DRIVE_LIMITER.calculate(driver.getRawAxis(Constants.kControls.TRANSLATION_Y_AXIS)),
      () -> -Constants.kControls.Y_DRIVE_LIMITER.calculate(driver.getRawAxis(Constants.kControls.TRANSLATION_X_AXIS)), 
      () -> -Constants.kControls.THETA_DRIVE_LIMITER.calculate(driver.getRawAxis(Constants.kControls.ROTATION_AXIS)),
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
    return auto.getSelectedCommand();
  }
}
