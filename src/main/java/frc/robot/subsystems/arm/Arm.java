package frc.robot.subsystems.arm;

import static frc.robot.Constants.ADVANTAGE_KIT_ENABLED;
import static frc.robot.Constants.TAB_MAIN;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team3061.RobotConfig;
import frc.lib.team6328.util.Alert;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {

  public static final int HOME_ROTATION = 0;
  public static final int HOME_EXTENSION = 1;

  static final double HEIGHT_OFFSET = RobotConfig.getInstance().getRobotArmHeightOffset();
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private static final Alert invalidAngle =
      new Alert("Invalid Angle Reached! (Arm Kinematics)", Alert.AlertType.ERROR);

  public Arm(ArmIO io) {
    this.io = io;
    TAB_MAIN.addNumber("Arm Angle", io::getArmAngle).withPosition(9, 1);
    TAB_MAIN.addNumber("Arm Length", io::getArmLength).withPosition(9, 2);
  }

  public void setArmAngle(double degrees) {
    io.setArmAngle(degrees);
  }

  public void setArmLength(double inches) {
    io.setArmLength(inches);
  }

  public double getArmAngle() {
    return io.getArmAngle();
  }

  public double getArmLength() {
    return io.getArmLength();
  }

  public void holdCurrentAngle() {
    io.holdCurrentAngle();
  }

  public void holdCurrentAngle(double desiredPosition) {
    io.holdCurrentAngle(desiredPosition);
  }

  public void stallArm() {
    io.stallArm();
  }

  public void stop() {
    io.stop();
  }

  public void setExtensionNominal() {
    io.setExtensionNominal();
  }

  public void zeroExtension() {
    io.zeroExtension();
  }

  @Override
  public void periodic() {
    if (ADVANTAGE_KIT_ENABLED) {
      io.updateInputs(inputs);
      Logger.getInstance().processInputs("Arm", inputs);
    }
  }

  public static double calculateArmLength(double x, double y) {
    return Math.sqrt(Math.pow(x, 2) + Math.pow((y - HEIGHT_OFFSET), 2));
  }

  public static Translation2d calculateArmPositionXy(double angle, double armLength) {
    double angleRad = Math.toRadians(angle);
    double x = armLength * Math.cos(angleRad);
    double y = armLength * Math.sin(angleRad) + HEIGHT_OFFSET;
    return new Translation2d(x, y);
  }

  public static double calculateArmAngle(double x, double y) {
    if (x == 0 && y > HEIGHT_OFFSET) return 90.0;
    if (x == 0 && y < HEIGHT_OFFSET) invalidAngle.set(true);
    if (x == 0) x = 0.1; // Safety for VERY rare case (likely don't need but safer)
    double v = Math.toDegrees(Math.atan((y - HEIGHT_OFFSET) / x));
    if (x > 0) return v;
    if (x < 0) return 180 + v;
    return 0.0;
  }
}
