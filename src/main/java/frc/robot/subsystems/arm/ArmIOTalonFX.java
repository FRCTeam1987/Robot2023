package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.AnalogInput;

public class ArmIOTalonFX implements ArmIO {
  private TalonFX rotationLeader;
  private TalonFX rotationFollower;
  private CANCoder rotationEncoder;
  private TalonFX telescopingMotor;

  private AnalogInput telescopePotentiometer;

  double temporaryArmCancoderOffset = 248.906;

  public ArmIOTalonFX(
      int leaderMotorID,
      int followerMotorID,
      int rotationCANCoderID,
      int telescopingMotorID,
      int potAnalogInputID,
      String canBusName) {
    rotationLeader = new TalonFX(leaderMotorID, canBusName);
    rotationLeader.configFactoryDefault();
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.remoteFilter0.remoteSensorDeviceID = rotationCANCoderID;
    config.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
    config.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
    config.slot0.kP = 1;
    config.slot0.kI = 0.0;
    config.slot0.kD = 0.5;
    config.slot0.kF = 0.0;
    config.slot0.allowableClosedloopError = 0;
    rotationLeader.configAllSettings(config);

    rotationFollower = new TalonFX(followerMotorID, canBusName);
    rotationFollower.configFactoryDefault();
    rotationFollower.follow(rotationLeader);
    
    
    rotationEncoder = new CANCoder(rotationCANCoderID, canBusName);
    rotationEncoder.configFactoryDefault();
    
    
    telescopingMotor = new TalonFX(telescopingMotorID);
    telescopingMotor.configFactoryDefault();

    telescopePotentiometer = new AnalogInput(potAnalogInputID);

  }

  public double getArmLength() {
    return telescopePotentiometer.getVoltage();
    //TODO: make this work
  }
  @Override
  public double getEncoderPosition() {
    return rotationEncoder.getAbsolutePosition() - temporaryArmCancoderOffset;
  }

  public void extendArmToLength(double length) {
    telescopingMotor.set(TalonFXControlMode.Position, length);
  }
  @Override
  public void rotateArmToAngle(double angle) {
    rotationLeader.set(TalonFXControlMode.Position, angle - temporaryArmCancoderOffset);
  }

  @Override
  public void setSpeedRot(double speed) {
    rotationLeader.set(TalonFXControlMode.PercentOutput, speed);
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
    inputs.armAbsoluteAngle = rotationEncoder.getAbsolutePosition();
  }
}
