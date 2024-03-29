/*
 * Initially from https://github.com/frc1678/C2022
 */

package frc.lib.team254.drivers;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

@java.lang.SuppressWarnings({"java:S1104", "java:S116"})

/**
 * Creates CANTalon objects and configures all the parameters we care about to factory defaults.
 * Closed-loop and sensor parameters are not set, as these are expected to be set by the
 * application.
 */
public class TalonFXFactory {

  private static final int TIMEOUT_MS = 100;

  // These periods don't share any common factors, so they shouldn't run at the same time. 255 is
  // max. (initially from https://github.com/Mechanical-Advantage/RobotCode2022)
  private static final int[] PRIME_PERIODS =
      new int[] {255, 254, 253, 251, 247, 241, 239, 233, 229, 227, 223, 217, 211, 199, 197};

  public static class Configuration {
    public NeutralMode NEUTRAL_MODE = NeutralMode.Coast;

    // factory default
    public final double NEUTRAL_DEADBAND = 0.04;

    public boolean ENABLE_SOFT_LIMIT = false;
    public final boolean ENABLE_LIMIT_SWITCH = false;
    public final int FORWARD_SOFT_LIMIT = 0;
    public final int REVERSE_SOFT_LIMIT = 0;

    public boolean INVERTED = false;
    public final boolean SENSOR_PHASE = false;
    public final SensorInitializationStrategy SENSOR_INITIALIZATION_STRATEGY =
        SensorInitializationStrategy.BootToZero;

    public int CONTROL_FRAME_PERIOD_MS = 20; // 10
    public int MOTION_CONTROL_FRAME_PERIOD_MS = 100;

    public int GENERAL_STATUS_FRAME_RATE_MS = 10;
    public int FEEDBACK_STATUS_FRAME_RATE_MS = 49;
    public int QUAD_ENCODER_STATUS_FRAME_RATE_MS = PRIME_PERIODS[0];
    public int ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = PRIME_PERIODS[1];
    public int PULSE_WIDTH_STATUS_FRAME_RATE_MS = PRIME_PERIODS[2];
    public int MOTION_MAGIC_STATUS_FRAME_RATE_MS = PRIME_PERIODS[3];
    public final int FEEDBACK_1_STATUS_FRAME_RATE_MS = PRIME_PERIODS[4];
    public int BASE_PIDF0_STATUS_FRAME_RATE_MS = PRIME_PERIODS[5];
    public final int TURN_PIDF1_STATUS_FRAME_RATE_MS = PRIME_PERIODS[6];
    public final int FEEDBACK_INTEGRATED_STATUS_FRAME_RATE_MS = PRIME_PERIODS[7];

    public final SensorVelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD =
        SensorVelocityMeasPeriod.Period_100Ms;
    public final int VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 64;

    public final StatorCurrentLimitConfiguration STATOR_CURRENT_LIMIT =
        new StatorCurrentLimitConfiguration(false, 300, 700, 1);
    public SupplyCurrentLimitConfiguration SUPPLY_CURRENT_LIMIT =
        new SupplyCurrentLimitConfiguration(false, 40, 100, 1);

    public double OPEN_LOOP_RAMP_RATE = 0.0;
    public double CLOSED_LOOP_RAMP_RATE = 0.0;

    public double SLOT0_KP = 0.0;
    public double SLOT0_KI = 0.0;
    public double SLOT0_KD = 0.0;
    public double SLOT0_KF = 0.0;
  }

  private static final Configuration DEFAULT_CONFIGURATION = new Configuration();
  private static final Configuration FOLLOWER_CONFIGURATION = new Configuration();

  static {
    // This control frame value seems to need to be something reasonable to avoid the Talon's
    // LEDs behaving erratically. Potentially try to increase as much as possible.
    FOLLOWER_CONFIGURATION.CONTROL_FRAME_PERIOD_MS = 100;
    FOLLOWER_CONFIGURATION.MOTION_CONTROL_FRAME_PERIOD_MS = 1000;
    FOLLOWER_CONFIGURATION.GENERAL_STATUS_FRAME_RATE_MS = 1000;
    FOLLOWER_CONFIGURATION.FEEDBACK_STATUS_FRAME_RATE_MS = 1000;
    FOLLOWER_CONFIGURATION.QUAD_ENCODER_STATUS_FRAME_RATE_MS = 1000;
    FOLLOWER_CONFIGURATION.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 1000;
    FOLLOWER_CONFIGURATION.PULSE_WIDTH_STATUS_FRAME_RATE_MS = 1000;
    FOLLOWER_CONFIGURATION.ENABLE_SOFT_LIMIT = false;
  }

  // create a CANTalon with the default (out of the box) configuration
  public static TalonFX createDefaultTalon(int id, String canBusName) {
    return createTalon(id, canBusName, DEFAULT_CONFIGURATION);
  }

  public static TalonFX createPermanentFollowerTalon(int id, String canBusName, int leaderID) {
    final TalonFX talon = createTalon(id, canBusName, FOLLOWER_CONFIGURATION);
    talon.set(ControlMode.Follower, leaderID);
    return talon;
  }

  public static TalonFX createTalon(int id, String canBusName, Configuration config) {
    TalonFX talon = new TalonFX(id, canBusName);
    TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();

    talon.configFactoryDefault();

    talon.set(ControlMode.PercentOutput, 0.0);

    talonFXConfig.forwardLimitSwitchSource = LimitSwitchSource.FeedbackConnector;
    talonFXConfig.forwardLimitSwitchNormal = LimitSwitchNormal.Disabled;
    talonFXConfig.reverseLimitSwitchSource = LimitSwitchSource.FeedbackConnector;
    talonFXConfig.reverseLimitSwitchNormal = LimitSwitchNormal.Disabled;

    talonFXConfig.clearPositionOnLimitF = false;
    talonFXConfig.clearPositionOnLimitR = false;

    talonFXConfig.nominalOutputForward = 0.0;
    talonFXConfig.nominalOutputReverse = 0.0;
    talonFXConfig.neutralDeadband = config.NEUTRAL_DEADBAND;

    talonFXConfig.peakOutputForward = 1.0;
    talonFXConfig.peakOutputReverse = -1.0;

    talonFXConfig.forwardSoftLimitThreshold = config.FORWARD_SOFT_LIMIT;
    talonFXConfig.forwardSoftLimitEnable = config.ENABLE_SOFT_LIMIT;
    talonFXConfig.reverseSoftLimitThreshold = config.REVERSE_SOFT_LIMIT;
    talonFXConfig.reverseSoftLimitEnable = config.ENABLE_SOFT_LIMIT;

    talonFXConfig.initializationStrategy = config.SENSOR_INITIALIZATION_STRATEGY;

    talonFXConfig.velocityMeasurementPeriod = config.VELOCITY_MEASUREMENT_PERIOD;
    talonFXConfig.velocityMeasurementWindow = config.VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW;

    talonFXConfig.openloopRamp = config.OPEN_LOOP_RAMP_RATE;
    talonFXConfig.closedloopRamp = config.CLOSED_LOOP_RAMP_RATE;

    talonFXConfig.voltageCompSaturation = 0.0;
    talonFXConfig.voltageMeasurementFilter = 32;

    talonFXConfig.statorCurrLimit = config.STATOR_CURRENT_LIMIT;
    talonFXConfig.supplyCurrLimit = config.SUPPLY_CURRENT_LIMIT;

    talonFXConfig.slot0.kP = config.SLOT0_KP;
    talonFXConfig.slot0.kI = config.SLOT0_KI;
    talonFXConfig.slot0.kD = config.SLOT0_KD;
    talonFXConfig.slot0.kF = config.SLOT0_KF;

    talon.configAllSettings(talonFXConfig, TIMEOUT_MS);

    talon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    talon.changeMotionControlFramePeriod(config.MOTION_CONTROL_FRAME_PERIOD_MS);
    talon.clearMotionProfileHasUnderrun(TIMEOUT_MS);
    talon.clearMotionProfileTrajectories();

    talon.clearStickyFaults(TIMEOUT_MS);

    talon.overrideLimitSwitchesEnable(config.ENABLE_LIMIT_SWITCH);

    talon.setNeutralMode(config.NEUTRAL_MODE);

    talon.overrideSoftLimitsEnable(config.ENABLE_SOFT_LIMIT);

    talon.setInverted(config.INVERTED);
    talon.setSensorPhase(config.SENSOR_PHASE);

    talon.selectProfileSlot(0, 0);

    talon.enableVoltageCompensation(false);

    talon.setStatusFramePeriod(
        StatusFrameEnhanced.Status_1_General, config.GENERAL_STATUS_FRAME_RATE_MS, TIMEOUT_MS);
    talon.setStatusFramePeriod(
        StatusFrameEnhanced.Status_2_Feedback0, config.FEEDBACK_STATUS_FRAME_RATE_MS, TIMEOUT_MS);
    talon.setStatusFramePeriod(
        StatusFrameEnhanced.Status_3_Quadrature,
        config.QUAD_ENCODER_STATUS_FRAME_RATE_MS,
        TIMEOUT_MS);
    talon.setStatusFramePeriod(
        StatusFrameEnhanced.Status_4_AinTempVbat,
        config.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS,
        TIMEOUT_MS);
    talon.setStatusFramePeriod(
        StatusFrameEnhanced.Status_8_PulseWidth,
        config.PULSE_WIDTH_STATUS_FRAME_RATE_MS,
        TIMEOUT_MS);
    talon.setStatusFramePeriod(
        StatusFrameEnhanced.Status_10_MotionMagic,
        config.MOTION_MAGIC_STATUS_FRAME_RATE_MS,
        TIMEOUT_MS);
    talon.setStatusFramePeriod(
        StatusFrameEnhanced.Status_12_Feedback1,
        config.FEEDBACK_1_STATUS_FRAME_RATE_MS,
        TIMEOUT_MS);
    talon.setStatusFramePeriod(
        StatusFrameEnhanced.Status_13_Base_PIDF0,
        config.BASE_PIDF0_STATUS_FRAME_RATE_MS,
        TIMEOUT_MS);
    talon.setStatusFramePeriod(
        StatusFrameEnhanced.Status_14_Turn_PIDF1,
        config.TURN_PIDF1_STATUS_FRAME_RATE_MS,
        TIMEOUT_MS);
    talon.setStatusFramePeriod(
        StatusFrameEnhanced.Status_21_FeedbackIntegrated,
        config.FEEDBACK_INTEGRATED_STATUS_FRAME_RATE_MS,
        TIMEOUT_MS);

    talon.setControlFramePeriod(ControlFrame.Control_3_General, config.CONTROL_FRAME_PERIOD_MS);

    return talon;
  }

  private TalonFXFactory() {}
}
