
package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import frc.robot.subsystems.Swerve;

/**
 * This class generates auto commands.
 */
public class AutoCommands {
  private final Swerve swerve;
  private final SendableChooser<Command> dropDown;

  /**
   * Define all auto commands.
   */
  public AutoCommands(Swerve swerve) {
    this.swerve = swerve;

    dropDown = new SendableChooser<>();
    dropDown.addOption("Example", run(() -> {
      getCommand("Test", true);
    }));

    SmartDashboard.putData("Auto Selection", dropDown);
  }

  public Command getSelectedCommand() {
    return dropDown.getSelected();
  }

  private Command getCommand(String pathName, boolean isFirstPath) {
    PathPlannerTrajectory traj = PathPlanner.loadPath(
      pathName,
      Constants.kAuto.MAX_VELOCITY_METERS_PER_SECOND,
      Constants.kAuto.MAX_ACCEL_METERS_PER_SECOND_SQUARED);

    return sequence(
      
      run(() -> {
        if (isFirstPath) {
          swerve.resetOdometry(traj.getInitialHolonomicPose());
        }
      }, swerve),

      new PPSwerveControllerCommand(
        traj,
        swerve::getPose,
        Constants.kSwerve.KINEMATICS,
        new PIDController(Constants.kAuto.X_CONTROLLER_KP, 0, 0),
        new PIDController(Constants.kAuto.Y_CONTROLLER_KP, 0, 0),
        new PIDController(Constants.kAuto.THETA_CONTROLLER_KP, 0, 0),
        swerve::setModuleStates,
        swerve),

      swerve.drive(() -> 0.0, () -> 0.0, () -> 0.0, true, false));
  }
}