package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

public class ArmIOTalonFX {
  private TalonFX rotationLeader;
  private TalonFX rotationFollower;
  private CANCoder rotationEncoder;

  private TalonFX telescopingMotor;

  static double heightOffset = 12.0;

  public ArmIOTalonFX(
      int telescopingMotorID, int leaderMotorID, int followerMotorID, int rotationCANCoderID) {
    rotationLeader = new TalonFX(leaderMotorID);
    rotationFollower = new TalonFX(followerMotorID);
    rotationFollower.follow(rotationLeader);
    rotationEncoder = new CANCoder(rotationCANCoderID);

    telescopingMotor = new TalonFX(telescopingMotorID);
  }

  public static double calculateArmLength(double x, double y) {
    return Math.sqrt(Math.pow(x, 2) + Math.pow((y - heightOffset), 2));
  }

  public static double[] calculateArmPositionXy(double angle, double armLength) {
    double angleRad = Math.toRadians(angle);
    double x = armLength * Math.cos(angleRad);
    double y = armLength * Math.sin(angleRad) + heightOffset;
    return new double[] {x, y};
  }

  public static double calculateArmAngle(double x, double y) {
    if (x == 0 && y > heightOffset) return 90.0;
    if (x == 0 && y < heightOffset)
      System.out.println(
          "Impossible!"); // DriverStation.reportError("Impossible angle reached!", true);
    if (x > 0) return Math.toDegrees(Math.atan((y - heightOffset) / x));
    if (x < 0) return 180 + Math.toDegrees(Math.atan((y - heightOffset) / x));
    return 0.0;
  }
}
