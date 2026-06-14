package org.firstinspires.ftc.teamcode.pidautotuner;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

/**
 * Auto-tunes PID gains (in RPM units) for a velocity-controlled mechanism
 * (flywheel, intake, etc.) using the relay-feedback method for P/I/D and an
 * open-loop characterization sweep for the feedforward term F.
 *
 * <p>This version is written to plug directly into an FTCLib
 * {@link PIDController} using the common "ShooterConstants" pattern:
 * <pre>
 *     shooterRPMPID.setPID(kp, ki, kd);
 *     double power = shooterRPMPID.calculate(currentRPM, targetRPM);
 *     power += (targetRPM > 0) ? (kF * (targetRPM / MAX_RPM)) : 0.0;
 *     power = Range.clip(power, -1, 1);
 * </pre>
 *
 * <h2>Setup</h2>
 * <ol>
 *     <li>Edit {@link TuningConfig} -- set {@code MOTOR_NAME},
 *         {@code TICKS_PER_REV} (encoder ticks per output-shaft revolution,
 *         accounting for any gearing), {@code TARGET_RPM}, and check
 *         {@code REVERSED}.</li>
 *     <li>Run this OpMode. Press start.</li>
 *     <li><b>Phase 1 (relay test):</b> the motor oscillates its RPM around
 *         {@code TARGET_RPM}. This produces Ku and Tu.</li>
 *     <li><b>Phase 2 (feedforward sweep):</b> the motor runs open-loop at a
 *         few fixed power levels, measuring steady-state RPM at each. This
 *         produces kF -- both in this library's raw form (power per RPM) and
 *         in the normalized {@code kF * (target/MAX_RPM)} form used above.
 *         The sweep also reports the measured RPM at full power; copy that
 *         into {@code TuningConfig.MAX_RPM} for next time.</li>
 *     <li>Read the candidate kP/kI/kD/kF off the Driver Station telemetry.</li>
 *     <li>Hold gamepad1.a to live-test the "classic ZN" candidate using an
 *         FTCLib {@code PIDController}, exactly as you would in your final
 *         shooter code.</li>
 * </ol>
 */
@TeleOp(name = "PIDF Auto Tuner (Velocity) [Local]")
public class VelocityPIDFTunerOpMode extends LinearOpMode {

    /** Converts encoder ticks/sec to RPM using {@link TuningConfig#TICKS_PER_REV}. */
    private static double ticksPerSecToRPM(double ticksPerSec) {
        return (ticksPerSec / TuningConfig.TICKS_PER_REV) * 60.0;
    }

    @Override
    public void runOpMode() {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, TuningConfig.MOTOR_NAME);
        motor.setDirection(TuningConfig.REVERSED
                ? DcMotorSimple.Direction.REVERSE
                : DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("=== PIDF Auto Tuner (Velocity) [Local] ===");
        telemetry.addData("Motor", TuningConfig.MOTOR_NAME);
        telemetry.addData("Target RPM", "%.1f", TuningConfig.TARGET_RPM);
        telemetry.addLine("Press START. Phase 1: relay test (automatic).");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ---------------- Phase 1: relay test for Ku / Tu (in RPM) ----------------

        RelayAutoTuner tuner = new RelayAutoTuner(
                TuningConfig.TARGET_RPM,
                TuningConfig.RELAY_AMPLITUDE,
                TuningConfig.RPM_HYSTERESIS,
                TuningConfig.CYCLES_TO_COLLECT,
                TuningConfig.CYCLES_TO_IGNORE
        );

        ElapsedTime timer = new ElapsedTime();
        boolean timedOut = false;

        while (opModeIsActive() && !tuner.isFinished()) {
            double now = timer.seconds();
            double rpm = ticksPerSecToRPM(motor.getVelocity());
            double output = tuner.update(rpm, now);
            motor.setPower(output);

            telemetry.addLine("=== Phase 1: Relay Test ===");
            telemetry.addData("RPM", "%.1f", rpm);
            telemetry.addData("Target RPM", "%.1f", TuningConfig.TARGET_RPM);
            telemetry.addData("Cycles collected", "%d / %d", tuner.cyclesCollected(),
                    TuningConfig.CYCLES_TO_COLLECT + TuningConfig.CYCLES_TO_IGNORE);
            telemetry.addData("Elapsed (s)", "%.1f", now);
            telemetry.update();

            if (now > TuningConfig.RELAY_TEST_TIMEOUT_S) {
                timedOut = true;
                break;
            }
        }

        motor.setPower(0);
        sleep(500);

        if (timedOut) {
            telemetry.clearAll();
            telemetry.addLine("=== Tuning TIMED OUT ===");
            telemetry.addLine("The mechanism's RPM didn't complete enough");
            telemetry.addLine("oscillation cycles. Try increasing RELAY_AMPLITUDE,");
            telemetry.addLine("lowering TARGET_RPM, or check wiring/TICKS_PER_REV.");
            telemetry.update();
            while (opModeIsActive()) idle();
            return;
        }

        RelayTuningResult result = tuner.computeResult();
        if (result == null) {
            telemetry.clearAll();
            telemetry.addLine("=== Tuning FAILED ===");
            telemetry.addLine("Not enough oscillation amplitude was measured.");
            telemetry.update();
            while (opModeIsActive()) idle();
            return;
        }

        // ---------------- Phase 2: feedforward characterization (in RPM) ----------------

        FeedforwardCharacterizer ff = new FeedforwardCharacterizer();
        double measuredMaxRPM = 0;

        for (double power : TuningConfig.FEEDFORWARD_TEST_POWERS) {
            if (!opModeIsActive()) break;

            motor.setPower(power);
            ElapsedTime settleTimer = new ElapsedTime();
            double lastRPM = 0;

            while (opModeIsActive() && settleTimer.seconds() < TuningConfig.FEEDFORWARD_SETTLE_TIME_S) {
                lastRPM = ticksPerSecToRPM(motor.getVelocity());
                telemetry.addLine("=== Phase 2: Feedforward Sweep ===");
                telemetry.addData("Testing power", "%.2f", power);
                telemetry.addData("RPM", "%.1f", lastRPM);
                telemetry.addData("Settling", "%.1f / %.1f s",
                        settleTimer.seconds(), TuningConfig.FEEDFORWARD_SETTLE_TIME_S);
                telemetry.update();
            }

            ff.addSample(power, lastRPM);
            measuredMaxRPM = Math.max(measuredMaxRPM, lastRPM);
        }

        motor.setPower(0);
        sleep(500);

        // kFRaw: power = kFRaw * RPM (this library's units)
        double kFRaw = ff.computeKf();
        if (Double.isNaN(kFRaw)) {
            kFRaw = 0.0; // fall back to no feedforward if characterization failed
        }

        // kFNormalized: for `power += kF * (targetRPM / MAX_RPM)`, i.e. the
        // power fraction needed to sustain MAX_RPM.
        double kFNormalized = kFRaw * TuningConfig.MAX_RPM;

        List<PIDGains> candidates = ZieglerNicholsCalculator.computeCandidates(result, kFRaw, TuningConfig.TUNE_INTEGRAL_TERM);

        // Default to the "classic ZN" candidate for the live test.
        PIDGains liveTestGains = candidates.get(4);

        // FTCLib controller, used exactly like shooterRPMPID in your final code.
        PIDController shooterRPMPID = new PIDController(liveTestGains.kP, liveTestGains.kI, liveTestGains.kD);

        boolean liveTestRunning = false;

        while (opModeIsActive()) {
            telemetry.clearAll();
            telemetry.addLine("=== Relay Test Result ===");
            telemetry.addData("Ku", "%.6f", result.Ku);
            telemetry.addData("Tu (s)", "%.4f", result.Tu);
            telemetry.addLine();
            telemetry.addLine("=== Feedforward ===");
            telemetry.addData("Measured max RPM (at highest test power)", "%.1f", measuredMaxRPM);
            telemetry.addData("  -> copy this into TuningConfig.MAX_RPM", "");
            telemetry.addData("kF (raw, power per RPM)", "%.8f", kFRaw);
            telemetry.addData("kF (normalized, ShooterConstants.kF style)", "%.6f", kFNormalized);
            telemetry.addLine();
            telemetry.addLine("=== Candidate kP/kI/kD (RPM units) ===");
            for (PIDGains gains : candidates) {
                telemetry.addLine(gains.toString());
            }
            telemetry.addLine();
            telemetry.addLine("Copy a candidate's kP/kI/kD into ShooterConstants,");
            telemetry.addLine("along with kF (normalized) and MAX_RPM above.");
            telemetry.addLine("'classic ZN' is a good starting point for flywheels.");
            telemetry.addLine();
            telemetry.addLine("Hold A to live-test 'classic ZN' via FTCLib PIDController.");

            if (gamepad1.a) {
                if (!liveTestRunning) {
                    shooterRPMPID.reset();
                    liveTestRunning = true;
                }

                double topVelocity = Math.abs(motor.getVelocity());
                double currentRPM = ticksPerSecToRPM(topVelocity);
                double targetRPM = TuningConfig.TARGET_RPM;

                shooterRPMPID.setPIDF(liveTestGains.kP, liveTestGains.kI, liveTestGains.kD, 0.0);

                double power = shooterRPMPID.calculate(currentRPM, targetRPM);
                power += (targetRPM > 0) ? (kFNormalized * (targetRPM / TuningConfig.MAX_RPM)) : 0.0;
                power = Range.clip(power, -1.0, 1.0);

                motor.setPower(power);

                telemetry.addLine();
                telemetry.addLine("=== LIVE TEST ACTIVE (FTCLib PIDController) ===");
                telemetry.addData("Current RPM", "%.1f", currentRPM);
                telemetry.addData("Target RPM", "%.1f", targetRPM);
                telemetry.addData("Error (RPM)", "%.1f", targetRPM - currentRPM);
                telemetry.addData("Output power", "%.3f", power);
            } else {
                if (liveTestRunning) {
                    motor.setPower(0);
                    liveTestRunning = false;
                }
            }

            telemetry.update();
        }

        motor.setPower(0);
    }
}
