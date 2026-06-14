package org.firstinspires.ftc.teamcode.pidautotuner;

/**
 * Edit these values for your robot before running either tuner OpMode.
 * Keeping them in one place makes it easy to re-tune a different mechanism
 * just by changing this file.
 */
public final class TuningConfig {

    private TuningConfig() {}

    // ---- Hardware ------------------------------------------------------

    /** The name of the motor in your hardware configuration, e.g. "flywheel" or "arm". */
    public static final String MOTOR_NAME = "motor";

    /** Set true if positive power should move the mechanism toward a *lower* encoder reading. */
    public static final boolean REVERSED = false;

    // ---- Relay test tuning ---------------------------------------------

    /**
     * Magnitude of the relay (bang-bang) output, from 0 to 1. Pick the
     * smallest value that reliably overcomes friction/gravity and produces a
     * clean oscillation. 0.3-0.7 is typical. Too high can be violent or
     * unsafe for position mechanisms (arms/lifts) -- start low and increase
     * if the mechanism doesn't oscillate.
     */
    public static final double RELAY_AMPLITUDE = 0.5;

    /** Number of full oscillation cycles to average for the Ku/Tu calculation. */
    public static final int CYCLES_TO_COLLECT = 6;

    /** Number of initial cycles to discard while the oscillation settles. */
    public static final int CYCLES_TO_IGNORE = 2;

    /** Safety timeout for the relay test, in seconds, in case the system never oscillates. */
    public static final double RELAY_TEST_TIMEOUT_S = 15.0;

    // ---- Position tuner specific ----------------------------------------

    /**
     * Target position for the position PID tuner, in encoder ticks relative
     * to the position the motor is at when the OpMode starts.
     */
    public static final double POSITION_TARGET_TICKS = 400;

    /**
     * Hysteresis band for the position relay test, in encoder ticks. The
     * relay switches once the measured position crosses
     * target +/- this value. A few ticks is usually enough; increase if you
     * see rapid chatter instead of a clean oscillation.
     */
    public static final double POSITION_HYSTERESIS_TICKS = 10;

    // ---- Velocity (PIDF) tuner specific -----------------------------------

    /**
     * Target velocity for the velocity PIDF tuner, in encoder ticks per second.
     * Pick a value representative of where you'll actually run the mechanism
     * (e.g. your flywheel's shooting speed).
     */
    public static final double VELOCITY_TARGET_TICKS_PER_SEC = 1500;

    /** Hysteresis band for the velocity relay test, in ticks/sec. */
    public static final double VELOCITY_HYSTERESIS_TICKS_PER_SEC = 30;

    /**
     * Power levels to use for the feedforward (kF) characterization step.
     * The motor is run open-loop at each of these powers in turn, and the
     * steady-state velocity is recorded.
     */
    public static final double[] FEEDFORWARD_TEST_POWERS = {0.5, 0.75, 1.0};

    /** How long to hold each feedforward test power before sampling, in seconds. */
    public static final double FEEDFORWARD_SETTLE_TIME_S = 1.5;

    // ---- RPM units (FTCLib-style velocity tuning) --------------------------

    /**
     * Encoder ticks per revolution of the OUTPUT shaft you're measuring
     * (i.e. after any gearing). Used to convert ticks/sec <-> RPM so the
     * tuner's output gains can be plugged directly into a PIDController
     * that operates on RPM, e.g.:
     * <pre>
     *   shooterRPMPID.setPID(kp, ki, kd);
     *   double power = shooterRPMPID.calculate(currentRPM, targetRPM);
     *   power += (targetRPM > 0) ? (kF * (targetRPM / MAX_RPM)) : 0.0;
     * </pre>
     * For a bare motor this is its ticks-per-revolution (often called
     * cycles-per-revolution / CPR) spec. If there's external gearing after
     * the encoder, divide by the gear ratio (output revs per motor rev).
     */
    public static final double TICKS_PER_REV = 28.0; // e.g. REV HD Hex motor bare shaft CPR

    /**
     * Target velocity for the RPM-based velocity tuner, in RPM. This is the
     * RPM equivalent of {@link #VELOCITY_TARGET_TICKS_PER_SEC} and is what
     * the relay test will oscillate around.
     */
    public static final double TARGET_RPM = (VELOCITY_TARGET_TICKS_PER_SEC / TICKS_PER_REV) * 60.0;

    /** Hysteresis band for the RPM-based relay test, in RPM. */
    public static final double RPM_HYSTERESIS = (VELOCITY_HYSTERESIS_TICKS_PER_SEC / TICKS_PER_REV) * 60.0;

    /**
     * Maximum sustainable RPM at full power, used to convert this library's
     * raw kF (power per RPM) into the normalized
     * {@code power += kF * (targetRPM / MAX_RPM)} form used by the
     * ShooterConstants-style snippet above. Set this to your mechanism's
     * real measured max RPM -- the feedforward sweep will print a measured
     * value you can copy here for next time.
     */
    public static final double MAX_RPM = 6000; // placeholder; overwrite with your measured max RPM
}

