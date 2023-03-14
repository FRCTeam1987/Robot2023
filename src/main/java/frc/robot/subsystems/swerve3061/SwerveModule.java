/*
 * Initially from https://github.com/Team364/BaseFalconSwerve
 */

package frc.robot.subsystems.swerve3061;

import static frc.robot.Constants.*;
import static frc.robot.subsystems.constants.SwerveModuleConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.*;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** SwerveModule models a single swerve module. */
public class SwerveModule {
  private final int moduleNumber;
  private double lastAngle;
  private final double maxVelocity;
  private final double wheelCircumference;
  private final double driveGearRatio;
  private final boolean driveMotorInverted;
  private final double angleGearRatio;
  private final boolean angleMotorInverted;
  private final boolean canCoderInverted;
  private TalonFX mAngleMotor;
  private TalonFX mDriveMotor;
  private CANCoder angleEncoder;
  private final SimpleMotorFeedforward feedForward;
  private final double angleOffsetDeg;
  private static final boolean DEBUGGING = false;

  /**
   * Create a new swerve module.
   *
   * @param moduleNumber the module number (0-3)
   * @param maxVelocity the maximum drive velocity of the module in meters per second
   */
  public SwerveModule(
      int driveMotorID,
      int angleMotorID,
      int canCoderID,
      double angleOffsetDeg,
      int moduleNumber,
      double maxVelocity) {
    this.moduleNumber = moduleNumber;
    this.maxVelocity = maxVelocity;
    this.angleOffsetDeg = angleOffsetDeg;
    wheelCircumference = MK4I_L2_WHEEL_CIRCUMFERENCE;
    driveGearRatio = MK4I_L2_DRIVE_GEAR_RATIO;
    driveMotorInverted = MK4I_L2_DRIVE_MOTOR_INVERTED;
    angleGearRatio = MK4I_L2_ANGLE_GEAR_RATIO;
    angleMotorInverted = MK4I_L2_ANGLE_MOTOR_INVERTED;
    canCoderInverted = MK4I_L2_CAN_CODER_INVERTED;
    this.feedForward = new SimpleMotorFeedforward(DRIVE_KS / 12, DRIVE_KV / 12, DRIVE_KA / 12);
    configAngleEncoder(canCoderID);
    configAngleMotor(angleMotorID);
    configDriveMotor(driveMotorID);
    lastAngle = getState().angle.getDegrees();
    if (DEBUGGING) {
      TAB_DRIVETRAIN.addNumber(
          "Mod " + this.moduleNumber + ": Cancoder", this::getAngleAbsolutePositionDeg);
      TAB_DRIVETRAIN.addNumber(
          "Mod " + this.moduleNumber + ": Integrated", this::getAnglePositionDeg);
      TAB_DRIVETRAIN.addNumber(
          "Mod " + this.moduleNumber + ": Velocity", this::getDriveVelocityMetersPerSec);
    }
  }

  /**
   * Set this swerve module to the specified speed and angle.
   *
   * @param desiredState the desired state of the module
   * @param isOpenLoop if true, the drive motor will be set to the calculated fraction of the max
   *     velocity; if false, the drive motor will set to the specified velocity using a closed-loop
   *     controller (PID).
   * @param forceAngle if true, the module will be forced to rotate to the specified angle; if
   *     false, the module will not rotate if the velocity is less than 1% of the max velocity.
   */
  public void setDesiredState(
      SwerveModuleState desiredState, boolean isOpenLoop, boolean forceAngle) {

    // this optimization is specific to CTRE hardware; perhaps this responsibility should be demoted
    // to the hardware-specific classes.
    desiredState = optimize(desiredState, getState().angle);

    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / maxVelocity;
      setDriveMotorPercentage(percentOutput);
    } else {
      setDriveVelocity(desiredState.speedMetersPerSecond);
    }

    // Unless the angle is forced (e.g., X-stance), don't rotate the module if speed is less then
    // 1%. This prevents jittering if the controller isn't tuned perfectly. Perhaps more
    // importantly, it allows for smooth repeated movement as the wheel direction doesn't reset
    // during pauses (e.g., multi-segmented auto paths).
    double angle;
    if (!forceAngle && Math.abs(desiredState.speedMetersPerSecond) <= (maxVelocity * 0.01)) {
      angle = lastAngle;
    } else {
      angle = desiredState.angle.getDegrees();
    }

    setAnglePosition(angle);
    lastAngle = angle;
  }

  public void setVoltageForCharacterization(double voltage) {
    setAnglePosition(0.0);
    lastAngle = 0.0;
    setDriveMotorPercentage(voltage / 12.0);
  }

  public SwerveModuleState getState() {
    double velocity = getDriveVelocityMetersPerSec();
    Rotation2d angle = Rotation2d.fromDegrees(getAnglePositionDeg());
    return new SwerveModuleState(velocity, angle);
  }

  public SwerveModulePosition getPosition() {
    double distance = getDriveDistanceMeters();
    Rotation2d angle = Rotation2d.fromDegrees(getAnglePositionDeg());
    return new SwerveModulePosition(distance, angle);
  }

  public int getModuleNumber() {
    return moduleNumber;
  }

  private void configAngleEncoder(int canCoderID) {
    angleEncoder = new CANCoder(canCoderID, CAN_BUS_NAME);
    angleEncoder.configFactoryDefault();
    CANCoderConfiguration config = new CANCoderConfiguration();
    config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    config.sensorDirection = canCoderInverted;
    config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    config.sensorTimeBase = SensorTimeBase.PerSecond;
    angleEncoder.configAllSettings(config);
  }

  private void configAngleMotor(int angleMotorID) {
    TalonFXFactory.Configuration angleMotorConfig = new TalonFXFactory.Configuration();
    angleMotorConfig.SUPPLY_CURRENT_LIMIT =
        new SupplyCurrentLimitConfiguration(
            ANGLE_ENABLE_CURRENT_LIMIT,
            ANGLE_CONTINUOUS_CURRENT_LIMIT,
            ANGLE_PEAK_CURRENT_LIMIT,
            ANGLE_PEAK_CURRENT_DURATION);
    angleMotorConfig.INVERTED = angleMotorInverted;
    angleMotorConfig.NEUTRAL_MODE = ANGLE_NEUTRAL_MODE;
    angleMotorConfig.SLOT0_KP = ANGLE_KP;
    angleMotorConfig.SLOT0_KI = ANGLE_KI;
    angleMotorConfig.SLOT0_KD = ANGLE_KD;
    angleMotorConfig.SLOT0_KF = ANGLE_KF;
    angleMotorConfig.GENERAL_STATUS_FRAME_RATE_MS = 9;
    angleMotorConfig.FEEDBACK_STATUS_FRAME_RATE_MS = 19;
    angleMotorConfig.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 100;
    angleMotorConfig.MOTION_MAGIC_STATUS_FRAME_RATE_MS = 101;
    angleMotorConfig.BASE_PIDF0_STATUS_FRAME_RATE_MS = 102;
    mAngleMotor = TalonFXFactory.createTalon(angleMotorID, CAN_BUS_NAME, angleMotorConfig);
    double absolutePosition =
        degreesToFalcon(getCanCoder().getDegrees() - angleOffsetDeg, angleGearRatio);
    mAngleMotor.setSelectedSensorPosition(absolutePosition);
    mAngleMotor.configVoltageCompSaturation(12); // default 12v voltage compensation for motors
    mAngleMotor.enableVoltageCompensation(true);
  }

  private void configDriveMotor(int driveMotorID) {
    TalonFXFactory.Configuration driveMotorConfig = new TalonFXFactory.Configuration();
    driveMotorConfig.SUPPLY_CURRENT_LIMIT =
        new SupplyCurrentLimitConfiguration(
            DRIVE_ENABLE_CURRENT_LIMIT,
            DRIVE_CONTINUOUS_CURRENT_LIMIT,
            DRIVE_PEAK_CURRENT_LIMIT,
            DRIVE_PEAK_CURRENT_DURATION);
    driveMotorConfig.INVERTED = driveMotorInverted;
    driveMotorConfig.NEUTRAL_MODE = DRIVE_NEUTRAL_MODE;
    driveMotorConfig.OPEN_LOOP_RAMP_RATE = OPEN_LOOP_RAMP;
    driveMotorConfig.CLOSED_LOOP_RAMP_RATE = CLOSED_LOOP_RAMP;
    driveMotorConfig.SLOT0_KP = DRIVE_KP;
    driveMotorConfig.SLOT0_KI = DRIVE_KI;
    driveMotorConfig.SLOT0_KD = DRIVE_KD;
    driveMotorConfig.SLOT0_KF = DRIVE_KF;
    driveMotorConfig.GENERAL_STATUS_FRAME_RATE_MS = 9;
    driveMotorConfig.FEEDBACK_STATUS_FRAME_RATE_MS = 19;
    driveMotorConfig.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 100;
    driveMotorConfig.MOTION_MAGIC_STATUS_FRAME_RATE_MS = 101;
    driveMotorConfig.BASE_PIDF0_STATUS_FRAME_RATE_MS = 102;

    mDriveMotor = TalonFXFactory.createTalon(driveMotorID, CAN_BUS_NAME, driveMotorConfig);

    mDriveMotor.configVoltageCompSaturation(12); // default 12v voltage compensation for motors
    mDriveMotor.enableVoltageCompensation(true);
    mDriveMotor.setSelectedSensorPosition(0);
  }

  private Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
  }

  public void setDriveMotorPercentage(double percentage) {
    mDriveMotor.set(ControlMode.PercentOutput, percentage);
  }

  public void setDriveVelocity(double velocity) {
    double ticksPerSecond = mpsToFalcon(velocity, wheelCircumference, driveGearRatio);
    mDriveMotor.set(
        ControlMode.Velocity,
        ticksPerSecond,
        DemandType.ArbitraryFeedForward,
        calculateFeedforward(velocity));
  }

  private double calculateFeedforward(double velocity) {
    double percentage = this.feedForward.calculate(velocity);
    // clamp the voltage to the maximum voltage
    return Math.min(percentage, 1.0);
  }

  public void setAnglePosition(double degrees) {
    mAngleMotor.set(ControlMode.Position, degreesToFalcon(degrees, angleGearRatio));
  }

  public void setDriveBrakeMode(boolean enable) {
    mDriveMotor.setNeutralMode(enable ? NeutralMode.Brake : NeutralMode.Coast);
  }

  public void setAngleBrakeMode(boolean enable) {
    // always leave the angle motor in coast mode
    mAngleMotor.setNeutralMode(NeutralMode.Coast);
  }

  private double getDriveVelocityMetersPerSec() {
    return falconToMPS(mDriveMotor.getSelectedSensorVelocity(), wheelCircumference, driveGearRatio);
  }

  private double getDriveDistanceMeters() {
    return falconToMeters(
        mDriveMotor.getSelectedSensorPosition(), wheelCircumference, driveGearRatio);
  }

  private double getDrivePositionDeg() {
    return falconToDegrees(mDriveMotor.getSelectedSensorPosition(), driveGearRatio);
  }

  private double getDriveAppliedPercentage() {
    return mDriveMotor.getMotorOutputPercent();
  }

  private double getDriveCurrentAmps() {
    return mDriveMotor.getStatorCurrent();
  }

  private double getAngleAbsolutePositionDeg() {
    return angleEncoder.getAbsolutePosition();
  }

  private double getAnglePositionDeg() {
    return falconToDegrees(mAngleMotor.getSelectedSensorPosition(), angleGearRatio);
  }

  private double getAngleVelocityRevPerMin() {
    return falconToRPM(mAngleMotor.getSelectedSensorVelocity(), angleGearRatio);
  }

  private double getAngleAppliedPercentage() {
    return mAngleMotor.getMotorOutputPercent();
  }

  private double getAngleCurrentAmps() {
    return mAngleMotor.getStatorCurrent();
  }

  /**
   * Minimize the change in heading the desired swerve module state would require by potentially
   * reversing the direction the wheel spins. Customized from WPILib's version to include placing in
   * appropriate scope for CTRE onboard control.
   *
   * @param desiredState The desired state.
   * @param currentAngle The current module angle.
   */
  public static SwerveModuleState optimize(
      SwerveModuleState desiredState, Rotation2d currentAngle) {
    double targetAngle =
        placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
    double targetSpeed = desiredState.speedMetersPerSecond;
    double delta = targetAngle - currentAngle.getDegrees();
    if (Math.abs(delta) > 90) {
      targetSpeed = -targetSpeed;
      targetAngle = delta > 90 ? (targetAngle - 180) : (targetAngle + 180);
    }
    return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
  }

  /**
   * @param scopeReference Current Angle
   * @param newAngle Target Angle
   * @return Closest angle within scope
   */
  private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
    double lowerBound;
    double upperBound;
    double lowerOffset = scopeReference % 360;
    if (lowerOffset >= 0) {
      lowerBound = scopeReference - lowerOffset;
      upperBound = scopeReference + (360 - lowerOffset);
    } else {
      upperBound = scopeReference - lowerOffset;
      lowerBound = scopeReference - (360 + lowerOffset);
    }
    while (newAngle < lowerBound) {
      newAngle += 360;
    }
    while (newAngle > upperBound) {
      newAngle -= 360;
    }
    if (newAngle - scopeReference > 180) {
      newAngle -= 360;
    } else if (newAngle - scopeReference < -180) {
      newAngle += 360;
    }
    return newAngle;
  }

  /**
   * @param counts Falcon Counts
   * @param gearRatio Gear Ratio between Falcon and Mechanism
   * @return Degrees of Rotation of Mechanism
   */
  public static double falconToDegrees(double counts, double gearRatio) {
    return counts * (360.0 / (gearRatio * 2048.0));
  }

  /**
   * @param degrees Degrees of rotation of Mechanism
   * @param gearRatio Gear Ratio between Falcon and Mechanism
   * @return Falcon Counts
   */
  public static double degreesToFalcon(double degrees, double gearRatio) {
    return degrees / (360.0 / (gearRatio * 2048.0));
  }

  /**
   * @param velocityCounts Falcon Velocity Counts
   * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
   * @return RPM of Mechanism
   */
  public static double falconToRPM(double velocityCounts, double gearRatio) {
    double motorRPM = velocityCounts * (600.0 / 2048.0);
    return motorRPM / gearRatio;
  }

  /**
   * @param rpm RPM of mechanism
   * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
   * @return RPM of Mechanism
   */
  public static double rpmToFalcon(double rpm, double gearRatio) {
    double motorRPM = rpm * gearRatio;
    return motorRPM * (2048.0 / 600.0);
  }

  /**
   * @param counts Falcon counts
   * @param circumference Circumference of Wheel
   * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
   * @return Falcon Velocity Counts
   */
  public static double falconToMeters(double counts, double circumference, double gearRatio) {
    double motorRotations = counts / 2048.0;
    double wheelRotations = motorRotations / gearRatio;
    return (wheelRotations * circumference);
  }

  /**
   * @param velocitycounts Falcon Velocity Counts
   * @param circumference Circumference of Wheel
   * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
   * @return Falcon Velocity Counts
   */
  public static double falconToMPS(double velocitycounts, double circumference, double gearRatio) {
    double wheelRPM = falconToRPM(velocitycounts, gearRatio);
    return (wheelRPM * circumference) / 60;
  }

  /**
   * @param velocity Velocity MPS
   * @param circumference Circumference of Wheel
   * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
   * @return Falcon Velocity Counts
   */
  public static double mpsToFalcon(double velocity, double circumference, double gearRatio) {
    double wheelRPM = ((velocity * 60) / circumference);
    return rpmToFalcon(wheelRPM, gearRatio);
  }
}
