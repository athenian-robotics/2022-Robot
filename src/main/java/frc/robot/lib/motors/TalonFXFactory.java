package frc.robot.lib.motors;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

public class TalonFXFactory {

  private static final int timeoutMs = 100;

  public static class Configuration {
    public NeutralMode NEUTRAL_MODE = NeutralMode.Coast;
    // factory default
    public double NEUTRAL_DEADBAND = 0.04;

    public SensorInitializationStrategy SENSOR_INITIALIZATION_STRATEGY =
        SensorInitializationStrategy.BootToZero;
    public double SENSOR_OFFSET_DEGREES = 0;

    public boolean ENABLE_SUPPLY_CURRENT_LIMIT = false;
    public boolean ENABLE_STATOR_CURRENT_LIMIT = false;

    public boolean ENABLE_SOFT_LIMIT = false;
    public boolean ENABLE_LIMIT_SWITCH = false;
    public int FORWARD_SOFT_LIMIT = 0;
    public int REVERSE_SOFT_LIMIT = 0;

    public boolean INVERTED = false;
    public boolean SENSOR_PHASE = false;

    public int CONTROL_FRAME_PERIOD_MS = 10;
    public int MOTION_CONTROL_FRAME_PERIOD_MS = 1000;
    public int GENERAL_STATUS_FRAME_RATE_MS = 10;
    public int FEEDBACK_STATUS_FRAME_RATE_MS = 1000;
    public int QUAD_ENCODER_STATUS_FRAME_RATE_MS = 1000;
    public int ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 1000;
    public int PULSE_WIDTH_STATUS_FRAME_RATE_MS = 1000;

    public SensorVelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD =
        SensorVelocityMeasPeriod.Period_100Ms;
    public int VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 64;

    public double OPEN_LOOP_RAMP_RATE = 0.0;
    public double CLOSED_LOOP_RAMP_RATE = 0.0;
  }

  private static final Configuration defaultConfiguration = new Configuration();
  private static final Configuration slaveConfiguration = new Configuration();

  static {
    // This control frame value seems to need to be something reasonable to avoid the Talon's
    // LEDs behaving erratically. Potentially try to increase as much as possible.
    slaveConfiguration.CONTROL_FRAME_PERIOD_MS = 100;
    slaveConfiguration.MOTION_CONTROL_FRAME_PERIOD_MS = 1000;
    slaveConfiguration.GENERAL_STATUS_FRAME_RATE_MS = 1000;
    slaveConfiguration.FEEDBACK_STATUS_FRAME_RATE_MS = 1000;
    slaveConfiguration.QUAD_ENCODER_STATUS_FRAME_RATE_MS = 1000;
    slaveConfiguration.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 1000;
    slaveConfiguration.PULSE_WIDTH_STATUS_FRAME_RATE_MS = 1000;
    slaveConfiguration.ENABLE_SOFT_LIMIT = false;
  }

  // create a CANTalon with the default (out of the box) configuration
  public static WPI_TalonFX createDefaultTalon(int id) {
    return createTalon(id, defaultConfiguration);
  }

  public static WPI_TalonFX createPermanentSlaveTalon(int id, int master_id) {
    final WPI_TalonFX talon = createTalon(id, slaveConfiguration);
    talon.set(ControlMode.Follower, master_id);
    return talon;
  }

  public static WPI_TalonFX createTalon(int id, Configuration config) {
    WPI_TalonFX talon = new LazyTalonFX(id);
    talon.set(ControlMode.PercentOutput, 0.0);

    talon.changeMotionControlFramePeriod(config.MOTION_CONTROL_FRAME_PERIOD_MS);
    talon.clearMotionProfileHasUnderrun(timeoutMs);
    talon.clearMotionProfileTrajectories();

    talon.clearStickyFaults(timeoutMs);

    talon.configForwardLimitSwitchSource(
        LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, timeoutMs);
    talon.configReverseLimitSwitchSource(
        LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, timeoutMs);
    talon.overrideLimitSwitchesEnable(config.ENABLE_LIMIT_SWITCH);

    // Turn off re-zeroing by default.
    talon.configSetParameter(ParamEnum.eClearPositionOnLimitF, 0, 0, 0, timeoutMs);
    talon.configSetParameter(ParamEnum.eClearPositionOnLimitR, 0, 0, 0, timeoutMs);

    talon.configNominalOutputForward(0, timeoutMs);
    talon.configNominalOutputReverse(0, timeoutMs);
    talon.configNeutralDeadband(config.NEUTRAL_DEADBAND, timeoutMs);

    talon.configMotorCommutation(MotorCommutation.Trapezoidal);

    talon.configPeakOutputForward(1.0, timeoutMs);
    talon.configPeakOutputReverse(-1.0, timeoutMs);

    talon.setNeutralMode(config.NEUTRAL_MODE);

    talon.configForwardSoftLimitThreshold(config.FORWARD_SOFT_LIMIT, timeoutMs);
    talon.configForwardSoftLimitEnable(config.ENABLE_SOFT_LIMIT, timeoutMs);

    talon.configReverseSoftLimitThreshold(config.REVERSE_SOFT_LIMIT, timeoutMs);
    talon.configReverseSoftLimitEnable(config.ENABLE_SOFT_LIMIT, timeoutMs);
    talon.overrideSoftLimitsEnable(config.ENABLE_SOFT_LIMIT);

    talon.setInverted(config.INVERTED);
    talon.setSensorPhase(config.SENSOR_PHASE);

    talon.selectProfileSlot(0, 0);

    talon.configVelocityMeasurementPeriod(config.VELOCITY_MEASUREMENT_PERIOD, timeoutMs);

    talon.configVelocityMeasurementWindow(
        config.VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW, timeoutMs);

    talon.configOpenloopRamp(config.OPEN_LOOP_RAMP_RATE, timeoutMs);
    talon.configClosedloopRamp(config.CLOSED_LOOP_RAMP_RATE, timeoutMs);

    talon.configVoltageCompSaturation(0.0, timeoutMs);
    talon.configVoltageMeasurementFilter(32, timeoutMs);
    talon.enableVoltageCompensation(false);

    talon.configSupplyCurrentLimit(
        new SupplyCurrentLimitConfiguration(config.ENABLE_SUPPLY_CURRENT_LIMIT, 20, 60, .2),
        timeoutMs);
    talon.configStatorCurrentLimit(
        new StatorCurrentLimitConfiguration(config.ENABLE_STATOR_CURRENT_LIMIT, 20, 60, .2),
        timeoutMs);

    talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, timeoutMs);
    talon.configIntegratedSensorInitializationStrategy(
        config.SENSOR_INITIALIZATION_STRATEGY, timeoutMs);
    talon.configIntegratedSensorOffset(config.SENSOR_OFFSET_DEGREES, timeoutMs);

    talon.setStatusFramePeriod(
        StatusFrameEnhanced.Status_1_General, config.GENERAL_STATUS_FRAME_RATE_MS, timeoutMs);
    talon.setStatusFramePeriod(
        StatusFrameEnhanced.Status_2_Feedback0, config.FEEDBACK_STATUS_FRAME_RATE_MS, timeoutMs);

    talon.setStatusFramePeriod(
        StatusFrameEnhanced.Status_3_Quadrature,
        config.QUAD_ENCODER_STATUS_FRAME_RATE_MS,
        timeoutMs);
    talon.setStatusFramePeriod(
        StatusFrameEnhanced.Status_4_AinTempVbat,
        config.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS,
        timeoutMs);
    talon.setStatusFramePeriod(
        StatusFrameEnhanced.Status_8_PulseWidth,
        config.PULSE_WIDTH_STATUS_FRAME_RATE_MS,
        timeoutMs);

    talon.setControlFramePeriod(ControlFrame.Control_3_General, config.CONTROL_FRAME_PERIOD_MS);

    return talon;
  }
}
