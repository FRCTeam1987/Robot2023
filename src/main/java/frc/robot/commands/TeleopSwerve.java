package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.team3061.RobotConfig;
import frc.robot.subsystems.drivetrain.Drivetrain;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * This command, when executed, instructs the drivetrain subsystem to drive based on the specified
 * values from the controller(s). This command is designed to be the default command for the
 * drivetrain subsystem.
 *
 * <p>Requires: the Drivetrain subsystem
 *
 * <p>Finished When: never
 *
 * <p>At End: stops the drivetrain
 */
public class TeleopSwerve extends CommandBase {

  private final Drivetrain drivetrain;
  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private final DoubleSupplier rotationSupplier;

  // TODO greyson play with these slew rate values
  private final SlewRateLimiter translationXSlewRate = new SlewRateLimiter(2.5);
  private final SlewRateLimiter translationYSlewRate = new SlewRateLimiter(2.5);
  private final SlewRateLimiter rotationSlewRate = new SlewRateLimiter(2);

  public static final double DEADBAND = 0.1;

  private final double maxVelocityMetersPerSecond = RobotConfig.getInstance().getRobotMaxVelocity();
  private final double maxAngularVelocityRadiansPerSecond =
      RobotConfig.getInstance().getRobotMaxAngularVelocity();

  /**
   * Create a new TeleopSwerve command object.
   *
   * @param drivetrain the drivetrain subsystem instructed by this command
   * @param translationXSupplier the supplier of the translation x value as a percentage of the
   *     maximum velocity as defined by the standard field or robot coordinate system
   * @param translationYSupplier the supplier of the translation y value as a percentage of the
   *     maximum velocity as defined by the standard field or robot coordinate system
   * @param rotationSupplier the supplier of the rotation value as a percentage of the maximum
   *     rotational velocity as defined by the standard field or robot coordinate system
   */
  public TeleopSwerve(
      Drivetrain drivetrain,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      DoubleSupplier rotationSupplier) {
    this.drivetrain = drivetrain;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.rotationSupplier = rotationSupplier;

    addRequirements(drivetrain);
  }

  @Override
  public void execute() {

    // invert the controller input and apply the deadband and squaring to make the robot more
    // responsive to small changes in the controller
    double xPercentage = translationXSlewRate.calculate(modifyAxis(translationXSupplier.getAsDouble()));
    double yPercentage = translationYSlewRate.calculate(modifyAxis(translationYSupplier.getAsDouble()));
    double rotationPercentage = rotationSlewRate.calculate(modifyAxis(rotationSupplier.getAsDouble()));

    double xVelocity = xPercentage * maxVelocityMetersPerSecond;
    double yVelocity = yPercentage * maxVelocityMetersPerSecond;
    double rotationalVelocity = rotationPercentage * maxAngularVelocityRadiansPerSecond;

    Logger.getInstance().recordOutput("ActiveCommands/TeleopSwerve", true);
    Logger.getInstance().recordOutput("TeleopSwerve/xVelocity", xVelocity);
    Logger.getInstance().recordOutput("TeleopSwerve/yVelocity", yVelocity);
    Logger.getInstance().recordOutput("TeleopSwerve/rotationalVelocity", rotationalVelocity);

    drivetrain.drive(xVelocity, yVelocity, rotationalVelocity, true);
  }

  @Override
  public void end(boolean interrupted) {
    this.drivetrain.stop();

    super.end(interrupted);

    Logger.getInstance().recordOutput("ActiveCommands/TeleopSwerve", false);
  }

  /**
   * Squares the specified value, while preserving the sign. This method is used on all joystick
   * inputs. This is useful as a non-linear range is more natural for the driver.
   *
   * @param value
   * @return
   */
  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, DEADBAND);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }
}
