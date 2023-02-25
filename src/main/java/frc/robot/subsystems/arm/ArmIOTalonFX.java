package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.team6328.util.Alert;
import java.util.Map;

public class ArmIOTalonFX implements ArmIO {
  private TalonFX rotMotorLeader;
  private TalonFX rotMotorFollower;
  private CANCoder rotEncoder;
  private TalonFX extMotor;
  private AnalogInput extPot;

  final double canCoderOffset = 202.939;

  final double maxRotAngle = 110;
  final double fullRotDegs = 360.0;
  final double fullRotTicks = 4096.0;
  final double minLengthInches = 0.0;
  final double maxLengthInches = 41.0;
  final double minMotorTicks = -161.0;
  final double maxMotorTicks = 89174.0;
  final double minPotVolts = 0.051269526;
  final double maxPotVolts = 4.44580032;

  final double conversionFactorDegsToTicks = (fullRotTicks) / (fullRotDegs);
  final double conversionFactorTicksToDegs = (fullRotDegs) / (fullRotTicks);
  final double conversionFactorInchesToTicks =
      (maxMotorTicks - minMotorTicks) / (maxLengthInches - minLengthInches);
  final double conversionFactorVoltsToInches =
      (maxLengthInches - minLengthInches) / (maxPotVolts - minPotVolts);

  private Alert armWentBeserkAlert =
      new Alert("Attempted to set arm beyond safe range.", Alert.AlertType.ERROR);

  // A couple abbrievations:
  //  - rot means rotation
  //  - ext means extension
  //  - pot means potentiometer

  public ArmIOTalonFX(
      int rotMotorLeaderID,
      int rotMotorFollowerID,
      int rotCANCoderID,
      int extMotorID,
      String canBusName) {

    rotEncoder = new CANCoder(rotCANCoderID, canBusName);
    rotEncoder.configFactoryDefault();
    rotEncoder.configAllSettings(getCanCoderConfig());
    rotEncoder.setPosition(rotEncoder.getAbsolutePosition() - canCoderOffset);

    rotMotorLeader = new TalonFX(rotMotorLeaderID, canBusName);
    rotMotorLeader.configFactoryDefault();
    rotMotorLeader.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 0, 100);
    rotMotorLeader.configVoltageCompSaturation(8);
    rotMotorLeader.configClosedloopRamp(0.5);
    rotMotorLeader.configAllSettings(getRotMotorConfig(rotCANCoderID));
    rotMotorLeader.setNeutralMode(NeutralMode.Brake);
    rotMotorLeader.enableVoltageCompensation(true);

    rotMotorFollower = new TalonFX(rotMotorFollowerID, canBusName);
    rotMotorFollower.configFactoryDefault();
    rotMotorFollower.configVoltageCompSaturation(8);
    rotMotorFollower.follow(rotMotorLeader);
    rotMotorFollower.setNeutralMode(NeutralMode.Brake);
    rotMotorLeader.enableVoltageCompensation(true);

    extMotor = new TalonFX(extMotorID);
    extMotor.configFactoryDefault();
    extMotor.configVoltageCompSaturation(4);
    extMotor.configClosedloopRamp(0.5);
    extMotor.configAllSettings(getExtMotorConfig());
    extMotor.setNeutralMode(NeutralMode.Brake);
    extMotor.enableVoltageCompensation(true);

    extPot = new AnalogInput(3);

    setShuffleBoardLayout();
  }

  private CANCoderConfiguration getCanCoderConfig() {
    CANCoderConfiguration canCoderConfig = new CANCoderConfiguration();
    canCoderConfig.sensorDirection = false;

    return canCoderConfig;
  }

  private TalonFXConfiguration getRotMotorConfig(int rotCANCoderID) {
    TalonFXConfiguration rotMotorConfig = new TalonFXConfiguration();
    rotMotorConfig.remoteFilter0.remoteSensorDeviceID = rotCANCoderID;
    rotMotorConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
    rotMotorConfig.feedbackNotContinuous = true;

    rotMotorConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
    rotMotorConfig.primaryPID.selectedFeedbackCoefficient = 1.0;
    rotMotorConfig.motionAcceleration = 1000;
    rotMotorConfig.motionCruiseVelocity = 1000;
    rotMotorConfig.slot0.kP = 2.25;
    rotMotorConfig.slot0.kI = 0.0;
    rotMotorConfig.slot0.kD = 0.0;
    rotMotorConfig.slot0.kF = 0.0;
    rotMotorConfig.slot0.allowableClosedloopError = 0;

    return rotMotorConfig;
  }

  private TalonFXConfiguration getExtMotorConfig() {
    TalonFXConfiguration extConfig = new TalonFXConfiguration();
    extConfig.feedbackNotContinuous = true;

    extConfig.primaryPID.selectedFeedbackCoefficient = 1.0;
    extConfig.slot0.kP = 0.1;
    extConfig.slot0.kI = 0.0;
    extConfig.slot0.kD = 0.0;
    extConfig.slot0.kF = 0.0;
    extConfig.slot0.allowableClosedloopError = 0;

    return extConfig;
  }

  private void setShuffleBoardLayout() {
    ShuffleboardTab armTab = Shuffleboard.getTab("Arm Tab");

    ShuffleboardLayout rotList =
        armTab
            .getLayout("Arm Rotation", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withProperties(Map.of("Label position", "HIDDEN"));

    GenericEntry targetAngle =
        rotList
            .add("Target Angle", 0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", minLengthInches, "max", maxLengthInches))
            .getEntry();
    rotList.add("Rotate Arm", new InstantCommand(() -> setArmAngle(targetAngle.getDouble(0))));
    rotList.addNumber("Arm Angle", (() -> getArmAngle()));
    rotList.addNumber("Rotation Motor Ticks", (() -> convertDegsToTicks(getArmAngle())));

    ShuffleboardLayout extList =
        armTab
            .getLayout("Arm Extension", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withProperties(Map.of("Label position", "HIDDEN"));

    GenericEntry targetLength =
        extList
            .add("Target Length", 0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", minLengthInches, "max", maxLengthInches))
            .getEntry();
    extList.add("Extend Arm", new InstantCommand(() -> setArmLength(targetLength.getDouble(0))));
    extList.addNumber("Arm Length Inches", (() -> convertVoltsToInches(extPot.getVoltage())));
    extList.addNumber("Extension Motor Ticks", (() -> extMotor.getSelectedSensorPosition()));
    extList.addNumber("Potentiometer Voltage", (() -> extPot.getVoltage()));

    armTab.add(
        "Coast Extension Motors",
        new InstantCommand(() -> extMotor.setNeutralMode(NeutralMode.Coast)));
    armTab.add(
        "Brake Extension Motors",
        new InstantCommand(() -> extMotor.setNeutralMode(NeutralMode.Brake)));
  }

  public int convertDegsToTicks(double degs) {
    return (int) (degs * conversionFactorDegsToTicks);
  }

  public double convertTicksToDegs(double ticks) {
    return (ticks * conversionFactorDegsToTicks);
  }

  private int convertInchesToTicks(double inches) {
    return (int) (inches * conversionFactorInchesToTicks);
  }

  private double convertVoltsToInches(double volts) {
    return (volts * conversionFactorVoltsToInches);
  }

  @Override
  public double getArmLength() {
    return (convertVoltsToInches(extPot.getVoltage() - minPotVolts));
  }

  @Override
  public void setArmLength(double inches) {
    if (minLengthInches < inches && inches < maxLengthInches) {
      extMotor.set(TalonFXControlMode.Position, convertInchesToTicks(inches));
    } else {
      armWentBeserkAlert.set(true);
    }
  }

  @Override
  public double getArmAngle() {
    return rotEncoder.getPosition();
  }

  @Override
  public void setArmAngle(double angle) {
    if (Math.abs(angle) < maxRotAngle) {
      rotMotorLeader.set(TalonFXControlMode.Position, convertDegsToTicks(angle));
    } else {
      armWentBeserkAlert.set(true);
    }
  }

  @Override
  public synchronized void updateInputs(ArmIOInputs inputs) {
    inputs.currentAmps =
        new double[] {
          rotMotorLeader.getStatorCurrent(),
          rotMotorFollower.getStatorCurrent(),
          extMotor.getStatorCurrent()
        };
    inputs.currentVolts =
        new double[] {
          rotMotorLeader.getMotorOutputVoltage(),
          rotMotorFollower.getMotorOutputVoltage(),
          extMotor.getMotorOutputVoltage()
        };
    inputs.armAbsoluteAngle = getArmAngle();
  }
}
