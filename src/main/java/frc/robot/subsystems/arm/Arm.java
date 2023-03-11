package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team3061.RobotConfig;
import frc.lib.team6328.util.Alert;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.ADVANTAGE_KIT_ENABLED;

public class Arm extends SubsystemBase {

  public static final int HOME_ROTATION = 0;
  public static final int HOME_EXTENSION = 1;

  static final double heightOffset = RobotConfig.getInstance().getRobotArmHeightOffset();
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private static final Alert invalidAngle =
      new Alert("Invalid Angle Reached! (Arm Kinematics)", Alert.AlertType.ERROR);

  final ShuffleboardTab tab = Shuffleboard.getTab("Arm Tab");

  public Arm(ArmIO io) {
    this.io = io;
    tab.addNumber("Arm Angle", io::getArmAngle);
    tab.addNumber("Arm Length", io::getArmLength);
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

  public void periodic() {
    if (ADVANTAGE_KIT_ENABLED) {
      io.updateInputs(inputs);
      Logger.getInstance().processInputs("Arm", inputs);
    }
  }

  public static double calculateArmLength(double x, double y) {
    return Math.sqrt(Math.pow(x, 2) + Math.pow((y - heightOffset), 2));
  }

  public static Translation2d calculateArmPositionXy(double angle, double armLength) {
    double angleRad = Math.toRadians(angle);
    double x = armLength * Math.cos(angleRad);
    double y = armLength * Math.sin(angleRad) + heightOffset;
    return new Translation2d(x, y);
  }

  public static double calculateArmAngle(double x, double y) {
    if (x == 0 && y > heightOffset) return 90.0;
    if (x == 0 && y < heightOffset) invalidAngle.set(true);
    double v = Math.toDegrees(Math.atan((y - heightOffset) / x));
    if (x > 0) return v;
    if (x < 0) return 180 + v;
    return 0.0;
  }
}
