package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.team6328.util.Alert;

public class ArmIOTalonFX implements ArmIO {
  private TalonFX rotationLeader;
  private TalonFX rotationFollower;
  private CANCoder rotationEncoder;
  private TalonFX telescopingMotor;
  private AnalogInput telescopePotentiometer;
  private Alert armWentBeserkAlert =
      new Alert("Attempted to set arm beyond safe range.", Alert.AlertType.ERROR);

  double temporaryArmCancoderOffset = 292.852;
  double armMinLength = 21;
  double armMaxLength = 62;
  double armMinPotLength = 0.220947243;
  double armMaxPotLength = 4.549560081;
  double armConversionConstant = (armMaxLength - armMinLength) / (armMaxPotLength - armMinPotLength);
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
    rotatorConfig.slot0.kP = 0.5;
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
    telescopeConfig.primaryPID.selectedFeedbackCoefficient = 1.0;
    telescopeConfig.feedbackNotContinuous = true;
    telescopeConfig.slot0.allowableClosedloopError = 0;
    telescopingMotor = new TalonFX(telescopingMotorID);
    telescopingMotor.configFactoryDefault();
    telescopingMotor.configAllSettings(telescopeConfig);

    telescopePotentiometer = new AnalogInput(3);
  }

  public int convertDegreesToTicks(double deg) {
    return (int) (deg * 4096) / 360;
  }

  public double convertTicksToDegrees(double ticks) {
    return 360 * (ticks / 4096);
  }
  // 4.549560081 = 60.5
  // 0.220947243 = 21
  // 4.328612838 + 0.220947243 = 39.5 + 21
  // 9.12532638
  @Override
  public double getArmLength() {
    return ((telescopePotentiometer.getVoltage() - armMinPotLength) * armConversionConstant) + armMinLength;
    // TODO: make this work
  }

  @Override
  public void setArmLength(double lengthInches) {
    if (!(lengthInches > armMaxLength || lengthInches < armMinLength)) {
      telescopingMotor.set(TalonFXControlMode.Position, convertDegreesToTicks(0.0));
    } else {
      armWentBeserkAlert.set(true);
    }
  }
  @Override
  public void setArmPower(double pwr) {
      telescopingMotor.set(TalonFXControlMode.PercentOutput, convertDegreesToTicks(pwr));
  }

  @Override
  public double getArmAngle() {
    return rotationEncoder.getAbsolutePosition() - temporaryArmCancoderOffset;
  }

  @Override
  public void setArmAngle(double angle) {
    if (!(Math.abs(angle) > armMaxAngleAbsolute)) {
      rotationLeader.set(TalonFXControlMode.Position, convertDegreesToTicks(angle));
    } else {
      armWentBeserkAlert.set(true);
    }
  }

  @Override
  public synchronized void updateInputs(ArmIOInputs inputs) {
    SlotConfiguration slot = new SlotConfiguration();
    slot.kP = SmartDashboard.getNumber("pidArm", 0.1);
    slot.kI = 0;
    slot.kD = 0;
    rotationLeader.configureSlot(slot, 0, 100);
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
  }
}
