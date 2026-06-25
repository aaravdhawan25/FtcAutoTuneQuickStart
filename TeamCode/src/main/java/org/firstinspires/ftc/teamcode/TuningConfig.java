package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

/**
 * Edit these values for your robot before running any tuner OpMode.
 *
 * <p>The {@code @Config} annotation makes every {@code public static}
 * field live-tunable via FTC Dashboard (http://192.168.43.1:8080/dash)
 * while the OpMode is running -- no redeploy needed.
 *
 * <p>This is the ONLY file you need to copy into your TeamCode.
 * Everything else (PIDMaster, DualMotorPIDMaster, the algorithms) comes
 * from the {@code pidautotuner-ftc} JitPack dependency.
 */
@Config
public class TuningConfig {

    private TuningConfig() {}

    // ---- Hardware ------------------------------------------------------

    /** The name of the motor in your hardware configuration, e.g. "shooter", "arm". */
    public static String MOTOR_NAME = "shooter";

    /** Set true if positive power should move the mechanism toward a *lower* encoder reading. */
    public static boolean REVERSED = false;

    // ---- Relay test tuning ---------------------------------------------

    /**
     * Magnitude of the relay (bang-bang) output, from 0 to 1.
     * 0.3-0.7 is typical. Too high can be violent for position mechanisms
     * -- start low and increase if the mechanism doesn't oscillate.
     */
    public static double RELAY_AMPLITUDE = 0.5;

    /** Number of full oscillation cycles to average for the Ku/Tu calculation. */
    public static int CYCLES_TO_COLLECT = 6;

    /** Number of initial cycles to discard while the oscillation settles. */
    public static int CYCLES_TO_IGNORE = 2;

    /** Safety timeout for the relay test in seconds. */
    public static double RELAY_TEST_TIMEOUT_S = 15.0;

    // ---- Position tuner specific ----------------------------------------

    /** Target offset in encoder ticks from start position (position tuner). */
    public static double POSITION_TARGET_TICKS = 400;

    /** Hysteresis deadband for the position relay test, in ticks. */
    public static double POSITION_HYSTERESIS_TICKS = 10;

    // ---- Velocity (PIDF) tuner specific -----------------------------------

    /**
     * Encoder ticks per revolution of the output shaft.
     *
     * Common values:
     *   GoBILDA 435 RPM  -> 384.5
     *   GoBILDA 312 RPM  -> 537.7
     *   GoBILDA 223 RPM  -> 751.8
     *   REV HD Hex bare  -> 28.0
     *   NeveRest 40      -> 1120.0
     */
    public static double TICKS_PER_REV = 28.0; // REV HD Hex bare shaft (common for shooters)

    /**
     * If true, use VELOCITY_TARGET_RPM as the tuning target (converted to
     * ticks/sec internally). If false, use VELOCITY_TARGET_TICKS_PER_SEC directly.
     */
    public static boolean USE_RPM_TARGET = false;

    /** Target velocity in RPM. Only used when USE_RPM_TARGET is true. */
    public static double VELOCITY_TARGET_RPM = 2800;

    /** Target velocity in ticks/sec. Used when USE_RPM_TARGET is false. */
    public static double VELOCITY_TARGET_TICKS_PER_SEC = 1500;

    /** Hysteresis deadband for the velocity relay test, in ticks/sec. */
    public static double VELOCITY_HYSTERESIS_TICKS_PER_SEC = 30;

    /** Power levels for the feedforward kF characterization sweep. */
    public static double[] FEEDFORWARD_TEST_POWERS = {0.5, 0.75, 1.0};

    /** Settle time per power level during the feedforward sweep, in seconds. */
    public static double FEEDFORWARD_SETTLE_TIME_S = 1.5;

    /**
     * If false, every candidate has kI = 0 and uses ZN PD-only rules.
     * Recommended for flywheels where kF handles steady-state error.
     */
    public static boolean TUNE_INTEGRAL_TERM = false;

    // ---- Dual motor velocity tuner specific --------------------------------

    /** Hardware config name of the second motor for the dual-velocity tuner. */
    public static String MOTOR_NAME_2 = "shooter2";

    /** Direction of the second motor (usually reversed on dual-flywheel setups). */
    public static boolean REVERSED_2 = true;

    /**
     * True if both motors have encoders. False if only motor 1 has an encoder
     * -- motor 2 is still driven but its velocity is not measured.
     */
    public static boolean DUAL_ENCODERS = false;

    // ---- Convenience helpers -----------------------------------------------

    /** Returns the effective velocity target in ticks/sec, respecting USE_RPM_TARGET. */
    public static double effectiveTargetTicksPerSec() {
        if (USE_RPM_TARGET) {
            return (VELOCITY_TARGET_RPM / 60.0) * TICKS_PER_REV;
        }
        return VELOCITY_TARGET_TICKS_PER_SEC;
    }

    /** Converts ticks/sec to RPM using TICKS_PER_REV. */
    public static double toRPM(double ticksPerSec) {
        return (ticksPerSec / TICKS_PER_REV) * 60.0;
    }

    /** Converts RPM to ticks/sec using TICKS_PER_REV. */
    public static double toTicksPerSec(double rpm) {
        return (rpm / 60.0) * TICKS_PER_REV;
    }
}
