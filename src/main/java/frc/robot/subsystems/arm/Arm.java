package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team3061.RobotConfig;
import frc.lib.team6328.util.Alert;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  static double heightOffset = RobotConfig.getInstance().getRobotArmHeightOffset();
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private static final Alert invalidAngle =
      new Alert("Invalid Angle Reached! (Arm Kinematics)", Alert.AlertType.ERROR);

  ShuffleboardTab tab = Shuffleboard.getTab("Arm Tab");

  public Arm(ArmIO io) {
    this.io = io;
    tab.addNumber("Arm Angle", io::getArmAngle);
    tab.addNumber("Arm Length", io::getArmLength);
  }

  public void setArmAngle(double degrees) {
    io.setArmAngle(degrees);
  }

  public void setArmLength(int inches) {
    io.setArmLength(inches);
  }

  public double getArmAngle() {
    return io.getArmAngle();
  }

  public double getArmLength() {
    return io.getArmLength();
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Arm", inputs);
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
    if (x > 0) return Math.toDegrees(Math.atan((y - heightOffset) / x));
    if (x < 0) return 180 + Math.toDegrees(Math.atan((y - heightOffset) / x));
    return 0.0;
  }
}
