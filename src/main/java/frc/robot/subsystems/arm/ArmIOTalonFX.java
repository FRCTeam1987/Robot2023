package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.team6328.util.Alert;

public class ArmIOTalonFX implements ArmIO {
  private TalonFX rotationLeader;
  private TalonFX rotationFollower;
  private CANCoder rotationEncoder;
  private TalonFX telescopingMotor;
  private AnalogInput telescopePotentiometer;
  private Alert armWentBeserkAlert =
      new Alert("Attempted to set arm beyond safe range.", Alert.AlertType.ERROR);

  double temporaryArmCancoderOffset = 202.939;
  double armMinLength = 0;
  double armMaxLength = 41;
  double armMaxTicks = 89174;
  double armMinTicks = -161;
  double armMinPotLength = 0.051269526;
  double armMaxPotLength = 4.44580032;
  double armConversionConstant =
      (armMaxLength - armMinLength) / (armMaxPotLength - armMinPotLength);
  double armConversionInchToTicks = 
      (armMaxTicks - armMinTicks) / (armMaxLength - armMinLength);
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
    rotatorConfig.motionAcceleration = 1000;
    rotatorConfig.motionCruiseVelocity = 1000;
    rotatorConfig.slot0.kP = 2.25;
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
    rotationLeader.configVoltageCompSaturation(8);
    rotationLeader.enableVoltageCompensation(true);
    rotationLeader.configClosedloopRamp(0.5);
    rotationFollower = new TalonFX(followerMotorID, canBusName);
    rotationFollower.configFactoryDefault();
    rotationFollower.follow(rotationLeader);
    rotationFollower.setNeutralMode(NeutralMode.Brake);
    rotationFollower.configVoltageCompSaturation(8);
    rotationLeader.enableVoltageCompensation(true);

    TalonFXConfiguration telescopeConfig = new TalonFXConfiguration();
    telescopeConfig.slot0.kP = 0.1;
    telescopeConfig.slot0.kI = 0.0;
    telescopeConfig.slot0.kD = 0.0;
    telescopeConfig.slot0.kF = 0.0;
    telescopeConfig.primaryPID.selectedFeedbackCoefficient = 1.0;
    telescopeConfig.feedbackNotContinuous = true;
    telescopeConfig.slot0.allowableClosedloopError = 0;
    telescopingMotor = new TalonFX(telescopingMotorID);
    telescopingMotor.configFactoryDefault();
    telescopingMotor.setNeutralMode(NeutralMode.Brake);
    telescopingMotor.configVoltageCompSaturation(4);
    telescopingMotor.enableVoltageCompensation(true);
    // telescopingMotor.configSupplyCurrentLimit(25);
    telescopingMotor.enableVoltageCompensation(true);
    telescopingMotor.configClosedloopRamp(0.5);
    telescopingMotor.configAllSettings(telescopeConfig);
    /*telescopingMotor.setSelectedSensorPosition(telescopingMotor.getSensorCollection().getIntegratedSensorAbsolutePosition() - 792);
    telescopingMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 100);
    telescopingMotor.configSetParameter(ParamEnum.eFeedbackNotContinuous, 32, 32, 32,32)*/
    Shuffleboard.getTab("ArmTab")
        .addNumber("ticksLen", telescopingMotor::getSelectedSensorPosition);
    telescopePotentiometer = new AnalogInput(3);
    Shuffleboard.getTab("ArmTab")
        .addNumber("Pot Rotation 2", (() -> telescopePotentiometer.getVoltage()));
    SmartDashboard.putNumber("Input Length", 31.0);
    Shuffleboard.getTab("ArmTab")
        .add(
            "setArmPosition",
            new InstantCommand(
                () -> {
                  setArmLength(SmartDashboard.getNumber("Input Length", 31));
                }));
  }

  private int convertInchesToTicks(double inches) {
    System.out.println(inches);
    System.out.println((int) (inches * armConversionInchToTicks));
    return (int) (inches * armConversionInchToTicks);
  }
  
  public int convertDegreesToTicks(double deg) {
    return (int) (deg * 4096) / 360;
  }

  public double convertTicksToDegrees(double ticks) {
    return 360 * (ticks / 4096);
  }

  public double getArmLengthTicks() {
    return telescopingMotor.getSelectedSensorPosition() - 792;
  }
  // 4.549560081 = 60.5
  // 0.220947243 = 21
  // 4.328612838 + 0.220947243 = 39.5 + 21
  // 9.12532638
  // 89136, 11

  @Override
  public double getArmLength() {
    return ((telescopePotentiometer.getVoltage() - armMinPotLength) * armConversionConstant)
        + armMinLength;
  }

  @Override
  public void setArmLength(double lengthInches) {
    if (!(lengthInches > armMaxLength || lengthInches < armMinLength)) {
      System.out.println("Current Position in Ticks: " + telescopingMotor.getSelectedSensorPosition());
      System.out.println("Length in Inches: " + lengthInches + "| Length in Ticks: " + convertInchesToTicks(lengthInches));
      telescopingMotor.set(TalonFXControlMode.Position, convertInchesToTicks(lengthInches));
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
