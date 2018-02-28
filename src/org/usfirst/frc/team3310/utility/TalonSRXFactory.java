package org.usfirst.frc.team3310.utility;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.MotorSafety;

/**
 * Creates TalonSRX objects and configures all the parameters we care about to factory defaults. Closed-loop and sensor
 * parameters are not set, as these are expected to be set by the application.
 */
public class TalonSRXFactory {

    public static class Configuration {
        public boolean LIMIT_SWITCH_NORMALLY_OPEN = true;
        public double MAX_OUTPUT = 1;
        public double NOMINAL_OUTPUT = 0;
        public double PEAK_OUTPUT = 12;
        public NeutralMode ENABLE_BRAKE = NeutralMode.Brake;
        public boolean ENABLE_CURRENT_LIMIT = false;
        public boolean ENABLE_SOFT_LIMIT = false;
        public boolean ENABLE_LIMIT_SWITCH = false;
        public int CURRENT_LIMIT = 0;
        public double EXPIRATION_TIMEOUT_SECONDS = MotorSafety.DEFAULT_SAFETY_EXPIRATION;
        public double FORWARD_SOFT_LIMIT = 0;
        public boolean INVERTED = false;
        public double NOMINAL_CLOSED_LOOP_VOLTAGE = 12;
        public double REVERSE_SOFT_LIMIT = 0;
        public boolean SAFETY_ENABLED = false;

        public int CONTROL_FRAME_PERIOD_MS = 5;
        public int MOTION_CONTROL_FRAME_PERIOD_MS = 100;
        public int GENERAL_STATUS_FRAME_RATE_MS = 5;
        public int FEEDBACK_STATUS_FRAME_RATE_MS = 100;
        public int QUAD_ENCODER_STATUS_FRAME_RATE_MS = 100;
        public int ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 100;
        public int PULSE_WIDTH_STATUS_FRAME_RATE_MS = 100;

        public VelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD = VelocityMeasPeriod.Period_100Ms;
        public int VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 64;

        public double VOLTAGE_COMPENSATION_RAMP_RATE = 0;
        public double VOLTAGE_RAMP_RATE = 0;
    }

    private static final Configuration kDefaultConfiguration = new Configuration();
    private static final Configuration kSlaveConfiguration = new Configuration();

    static {
        kSlaveConfiguration.CONTROL_FRAME_PERIOD_MS = 1000;
        kSlaveConfiguration.MOTION_CONTROL_FRAME_PERIOD_MS = 1000;
        kSlaveConfiguration.GENERAL_STATUS_FRAME_RATE_MS = 1000;
        kSlaveConfiguration.FEEDBACK_STATUS_FRAME_RATE_MS = 1000;
        kSlaveConfiguration.QUAD_ENCODER_STATUS_FRAME_RATE_MS = 1000;
        kSlaveConfiguration.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 1000;
        kSlaveConfiguration.PULSE_WIDTH_STATUS_FRAME_RATE_MS = 1000;
    }

    // Create a TalonSRX with the default (out of the box) configuration.
    public static TalonSRX createDefaultTalon(int id) {
        return createTalon(id, kDefaultConfiguration);
    }

    public static TalonSRXEncoder createTalonEncoder(int id, double encoderTicksToWorld, FeedbackDevice feedbackDevice) {
    	TalonSRXEncoder talon = new TalonSRXEncoder(id, encoderTicksToWorld, feedbackDevice);
//        initializeTalon(talon, kDefaultConfiguration);
        return talon;
    }

    public static TalonSRXEncoder createTalonEncoder(int id, double encoderTicksToWorld, boolean isRight, FeedbackDevice feedbackDevice) {
    	TalonSRXEncoder talon = new TalonSRXEncoder(id, encoderTicksToWorld, isRight, feedbackDevice);
//        initializeTalon(talon, kDefaultConfiguration);
        return talon;
    }

    public static TalonSRX createPermanentSlaveTalon(int id, int master_id) {
        final TalonSRX talon = createTalon(id, kSlaveConfiguration);
        talon.set(ControlMode.Follower, master_id);
        return talon;
    }

    public static TalonSRX createTalon(int id, Configuration config) {
        TalonSRX talon = new TalonSRX(id);
//        initializeTalon(talon, config);
        return talon;
    }
    
    public static void initializeTalon(TalonSRX talon, Configuration config) {
//        talon.setControlFramePeriod(config.CONTROL_FRAME_PERIOD_MS, TalonSRXEncoder.TIMEOUT_MS);
        talon.changeMotionControlFramePeriod(config.MOTION_CONTROL_FRAME_PERIOD_MS);
        talon.setIntegralAccumulator(0, TalonSRXEncoder.PID_IDX, TalonSRXEncoder.TIMEOUT_MS);
        talon.clearMotionProfileHasUnderrun(TalonSRXEncoder.TIMEOUT_MS);
        talon.clearMotionProfileTrajectories();
        talon.clearStickyFaults(TalonSRXEncoder.TIMEOUT_MS);
        talon.configPeakOutputForward(config.MAX_OUTPUT, TalonSRXEncoder.TIMEOUT_MS);
        talon.configPeakOutputReverse(-config.MAX_OUTPUT, TalonSRXEncoder.TIMEOUT_MS);
        talon.configNominalOutputForward(config.NOMINAL_OUTPUT, TalonSRXEncoder.TIMEOUT_MS);
        talon.configNominalOutputReverse(-config.NOMINAL_OUTPUT, TalonSRXEncoder.TIMEOUT_MS);
        talon.setNeutralMode(config.ENABLE_BRAKE);
        talon.enableCurrentLimit(config.ENABLE_CURRENT_LIMIT);
        talon.configForwardSoftLimitEnable(config.ENABLE_SOFT_LIMIT, TalonSRXEncoder.TIMEOUT_MS);
        talon.configReverseSoftLimitEnable(config.ENABLE_SOFT_LIMIT, TalonSRXEncoder.TIMEOUT_MS);
        talon.setInverted(config.INVERTED);
        talon.setSensorPhase(false);
        talon.setSelectedSensorPosition(0, TalonSRXEncoder.PID_IDX, TalonSRXEncoder.TIMEOUT_MS);
        talon.selectProfileSlot(0, TalonSRXEncoder.TIMEOUT_MS);
        talon.configVelocityMeasurementPeriod(config.VELOCITY_MEASUREMENT_PERIOD, TalonSRXEncoder.TIMEOUT_MS);
        talon.configVelocityMeasurementWindow(config.VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW, TalonSRXEncoder.TIMEOUT_MS);

        talon.setStatusFramePeriod(StatusFrame.Status_1_General, config.GENERAL_STATUS_FRAME_RATE_MS, TalonSRXEncoder.TIMEOUT_MS);
        talon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, config.FEEDBACK_STATUS_FRAME_RATE_MS, TalonSRXEncoder.TIMEOUT_MS);
        talon.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, config.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS, TalonSRXEncoder.TIMEOUT_MS);

//        talon.setStatusFrameRateMs(TalonSRX.StatusFrameRate.QuadEncoder, config.QUAD_ENCODER_STATUS_FRAME_RATE_MS);
//        talon.setStatusFrameRateMs(TalonSRX.StatusFrameRate.PulseWidth, config.PULSE_WIDTH_STATUS_FRAME_RATE_MS);
    }

 }
