package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import frc.lib.team6328.util.Alert;

public class ArmIOTalonFX implements ArmIO {
  private TalonFX rotationLeader;
  private TalonFX rotationFollower;
  private CANCoder rotationEncoder;
  private TalonFX telescopingMotor;
  private AnalogPotentiometer telescopePotentiometer;
  private Alert armWentBeserkAlert =
      new Alert("Attempted to set arm beyond safe range.", Alert.AlertType.ERROR);

  double temporaryArmCancoderOffset = 292.852;
  double armMinLength = 21;
  double armMaxLength = 62;
  double armMinPotLength = 0.2209474243;
  double armMaxPotLength = 4.549560081;
  double armConversionConstant =
      (armMaxLength - armMinLength) / (armMaxPotLength - armMinPotLength);
  double armMaxAngleAbsolute = 110;

  public ArmIOTalonFX(
      int leaderMotorID,
      int followerMotorID,
      int rotationCANCoderID,
      int telescopingMotorID,
      String canBusName) {

    CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
    canCoderConfiguration.sensorDirection = false;
    rotationEncoder = new CANCoder(rotationCANCoderID, canBusName);
    rotationEncoder.configFactoryDefault();
    rotationEncoder.configAllSettings(canCoderConfiguration);
    rotationEncoder.setPosition(rotationEncoder.getAbsolutePosition() - temporaryArmCancoderOffset);

    TalonFXConfiguration rotatorConfig = new TalonFXConfiguration();
    rotatorConfig.remoteFilter0.remoteSensorDeviceID = rotationCANCoderID;
    rotatorConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
    rotatorConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
    rotatorConfig.slot0.kP = 1.5;
    rotatorConfig.slot0.kI = 0.0;
    rotatorConfig.slot0.kD = 0.0;
    rotatorConfig.slot0.kF = 0.0;
    rotatorConfig.primaryPID.selectedFeedbackCoefficient = 1.0;
    rotatorConfig.feedbackNotContinuous = true;
    rotatorConfig.slot0.allowableClosedloopError = 0;
    rotationLeader = new TalonFX(leaderMotorID, canBusName);
    rotationLeader.configFactoryDefault();
    rotationLeader.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 0, 100);
    rotationLeader.configAllSettings(rotatorConfig);
    rotationLeader.setNeutralMode(NeutralMode.Brake);
    rotationFollower = new TalonFX(followerMotorID, canBusName);
    rotationFollower.configFactoryDefault();
    rotationFollower.follow(rotationLeader);
    rotationFollower.setNeutralMode(NeutralMode.Brake);

    TalonFXConfiguration telescopeConfig = new TalonFXConfiguration();
    telescopeConfig.slot0.kP = 1;
    telescopeConfig.slot0.kI = 0.0;
    telescopeConfig.slot0.kD = 0.0;
    telescopeConfig.slot0.kF = 0.0;

    telescopingMotor = new TalonFX(telescopingMotorID);
    telescopingMotor.configFactoryDefault();

    telescopePotentiometer = new AnalogPotentiometer(4, 40, 0);
  }

  public int convertDegreesToTicks(double deg) {
    return (int) (deg * 4096) / 360;
  }

  public double convertTicksToDegrees(double ticks) {
    return 360 * (ticks / 4096);
  }

  public double getArmLength() {
    try {
      return telescopePotentiometer.get();
    } catch (Exception e) {
      return 9.9;
    }
  }

  @Override
  public double getArmAngle() {
    return rotationEncoder.getAbsolutePosition() - temporaryArmCancoderOffset;
  }

  // TODO: make this work
  public void extendArmToLength(double length) {
    telescopingMotor.set(TalonFXControlMode.Position, length);
  }

  @Override
  public void setArmToAngle(double angle) {
    System.out.println("Function Called");
    if (!(Math.abs(angle) > 120)) {
      rotationLeader.set(TalonFXControlMode.Position, convertDegreesToTicks(angle));
      System.out.println("Angle: " + angle + " Ticks:  " + convertDegreesToTicks(angle));
    } else {
      armWentBeserkAlert.set(true);
    }
  }

  @Override
  public synchronized void updateInputs(ArmIOInputs inputs) {
    inputs.currentAmps =
        new double[] {
          rotationLeader.getStatorCurrent(),
          rotationFollower.getStatorCurrent(),
          telescopingMotor.getStatorCurrent()
        };
    inputs.currentVolts =
        new double[] {
          rotationLeader.getMotorOutputVoltage(),
          rotationFollower.getMotorOutputVoltage(),
          telescopingMotor.getMotorOutputVoltage()
        };
    inputs.armAbsoluteAngle = getArmAngle();
    inputs.armLength = new double[] {this.getArmLength()};
  }
}
