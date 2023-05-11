package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.team3061.RobotConfig;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.Util;
import java.util.NoSuchElementException;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
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

  private PIDController thetaController;
  private PIDController yController;
  private IntSupplier m_povDegree;
  private BooleanSupplier m_shouldYLock;
  private double m_ySetPoint = 0.0;
  private DoubleSupplier m_speedMultiplier;
  private boolean useDPad = false;
  private boolean useYLock = false;
  private double setPoint = 0.0;

  private final Drivetrain drivetrain;
  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private final DoubleSupplier rotationSupplier;

  // TODO greyson play with these slew rate values
  private final SlewRateLimiter translationXSlewRate = new SlewRateLimiter(2.5);
  private final SlewRateLimiter translationYSlewRate = new SlewRateLimiter(2.5);
  private final SlewRateLimiter rotationSlewRate = new SlewRateLimiter(2);

  public static final double DEADBAND = 0.05;

  public static final double maxVelocityMetersPerSecond =
      RobotConfig.getInstance().getRobotMaxVelocity();
  public static final double maxAngularVelocityRadiansPerSecond =
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
      DoubleSupplier rotationSupplier,
      DoubleSupplier speedMultiplier,
      IntSupplier povDegree,
      BooleanSupplier shouldYLock) {

    m_speedMultiplier = speedMultiplier;
    m_povDegree = povDegree;
    m_shouldYLock = shouldYLock;

    this.drivetrain = drivetrain;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.rotationSupplier = rotationSupplier;

    addRequirements(drivetrain);
    thetaController = new PIDController(0.01, 0.0, 0.0);
    thetaController.enableContinuousInput(-180, 180);
    yController = new PIDController(0.7, 0.0, 0.0);
    yController.enableContinuousInput(-1, 9);
  }

  @Override
  public void initialize() {
    useDPad = false;
    useYLock = false;
    setPoint = 0.0;
    m_ySetPoint = 0.0;
  }

  @Override
  public void execute() {
    // invert the controller input and apply the deadband and squaring to make the robot more
    // responsive to small changes in the controller
    double xPercentage =
        translationXSlewRate.calculate(modifyAxis(-translationXSupplier.getAsDouble()));
    double yPercentage =
        translationYSlewRate.calculate(modifyAxis(-translationYSupplier.getAsDouble()));
    double rotationPercentage =
        rotationSlewRate.calculate(modifyAxis(-rotationSupplier.getAsDouble()));

    if (useYLock && !Util.isWithinTolerance(translationYSupplier.getAsDouble(), 0.0, 0.5)) {
      useYLock = false;
      m_ySetPoint = 0.0;
      System.out.println("Stop using Y Lock.");
    } else if (m_shouldYLock.getAsBoolean() && drivetrain.getPoseX() < 2.5) {
      try {
        m_ySetPoint = drivetrain.getPose().nearest(Constants.OnTheFly.CONE_NODES_POSE).getY();
      } catch (NoSuchElementException e) {
        useYLock = false;
        DriverStation.reportWarning("Not close enough to cone node. Not locking.", false);
      }
    }
    if (useYLock) {
      yController.setSetpoint(m_ySetPoint);
      yPercentage = yController.calculate(drivetrain.getPose().getY(), yController.getSetpoint());
    }

    if (useDPad && !Util.isWithinTolerance(rotationSupplier.getAsDouble(), 0.0, 0.25)) {
      useDPad = false;
      setPoint = 0.0;
      System.out.println("Stop using DPad.");
    } else if (m_povDegree.getAsInt() >= 0) {
      useDPad = true;
      switch ((int) m_povDegree.getAsInt()) {
        case 0:
          setPoint = 0;
          break;
        case 90:
          setPoint = -90;
          break;
        case 180:
          setPoint = 180;
          break;
        case 270:
          setPoint = 90;
          break;
      }
    }
    if (useDPad) {
      thetaController.setSetpoint(setPoint);
      rotationPercentage =
          thetaController.calculate(
              drivetrain.getPose().getRotation().getDegrees(), thetaController.getSetpoint());
    }

    double xVelocity = xPercentage * maxVelocityMetersPerSecond * m_speedMultiplier.getAsDouble();
    double yVelocity = yPercentage * maxVelocityMetersPerSecond * m_speedMultiplier.getAsDouble();
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
   * @param value input value
   * @return square of the value
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
