package org.firstinspires.ftc.teamcode.TeleOp;

import android.util.Log;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.MotorPowers;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Shooter.ShotCalculator;
import org.firstinspires.ftc.teamcode.Shooter.ShotCounter;
import org.firstinspires.ftc.teamcode.ButtonMaps.WorkingBotButtonMap;

import java.io.File;
import java.util.Locale;

// Data for counting balls in (intake) and out (flywheel dips), and for checking how well the flywheel holds its target.
// Robot stays still. Intake and transfer run the whole time, the flywheel target moves like it does during an LT strafe.
// g1: dpad up/down = balls this trial, A = start trial, B = end trial, X = strafe RPM / fixed RPM
// Logcat tags: BallLog (one CSV row per loop), BallEvent (trial start/end), BallTrial (trial summary), BallSummary
// The same rows are saved to ball_count_log.csv in the FIRST/settings folder on stop
@TeleOp(name = "BallCountDataTest", group = "Tuning")
public class BallCountDataTest extends LinearOpMode {
    // ---------------- Tuning ----------------
    // Same powers as g1 right bumper shooting in WorkingBotButtonMap
    public static double INTAKE_POWER = WorkingBotButtonMap.SHOOT_INTAKE_POWER;
    public static double TRANSFER_POWER = WorkingBotButtonMap.SHOOT_TRANSFER_POWER;
    // Pretend strafe speed along the line, in/s, and how fast it turns around, in/s^2
    public static double SIM_STRAFE_SPEED = 22;
    public static double SIM_STRAFE_ACCEL = 60;
    // Target with X switched to fixed
    public static double FIXED_RPM = WorkingBotButtonMap.FIXED_RPM;
    // Each current read is its own hub command, about 2 ms each
    public static boolean READ_CURRENTS = true;
    public static double VOLTAGE_READ_SECONDS = 0.25;
    // Trials per ball count before it moves on to the next count
    public static int TRIALS_PER_COUNT = 5;
    // Tracking error is only counted this long after the flywheel leaves the ramp
    public static double SETTLE_TIME = 1.0;
    // Target slope that counts as rising / falling, RPM per second
    public static double SLOPE_RPM_PER_S = 2;
    // ----------------------------------------

    private static final String HEADER = "t,dt_ms,trial,balls,in_trial,mode,target,rpm,err,ramping,shoot_pos,"
            + "shoot_amps,intake_tps,intake_pos,intake_amps,volts,shots,sim_x,sim_vx,target_slope";
    private static final int MAX_FILE_ROWS = 60000;

    // Error stats for a stretch of loops
    private static class ErrorStats {
        int n;
        double sum, sumSq, maxAbs, minRpm = Double.POSITIVE_INFINITY, maxRpm = Double.NEGATIVE_INFINITY;

        void add(double err, double rpm) {
            n++;
            sum += err;
            sumSq += err * err;
            maxAbs = Math.max(maxAbs, Math.abs(err));
            minRpm = Math.min(minRpm, rpm);
            maxRpm = Math.max(maxRpm, rpm);
        }

        String describe() {
            if (n == 0) {
                return "no samples";
            }
            return String.format(Locale.US, "n=%d mean=%.2f rms=%.2f maxabs=%.2f rpm=%.1f..%.1f",
                    n, sum / n, Math.sqrt(sumSq / n), maxAbs, minRpm, maxRpm);
        }
    }

    private Robot robot;
    private DcMotorEx shootMotor;
    private DcMotorEx intakeMotor;
    private final StringBuilder row = new StringBuilder();
    private final StringBuilder file = new StringBuilder();
    private int fileRows = 0;

    // Pretend strafe along the blue strafe line
    private double simX, simVx, simAx, simHeading, simTurnRate;
    private int simDirection = 1;
    private boolean strafeMode = true;

    // Trials
    private int balls = 1;
    private int trial = 0;
    private boolean inTrial = false;
    private double trialStart = 0;
    private final int[] trialsDone = new int[5];
    private final ShotCounter shotCounter = new ShotCounter();
    private ErrorStats trialStats = new ErrorStats();
    private String lastTrialSummary = "none yet";

    // Tracking error with no balls going through, split by what the target is doing
    private final ErrorStats quietAll = new ErrorStats();
    private final ErrorStats quietRising = new ErrorStats();
    private final ErrorStats quietFalling = new ErrorStats();
    private final ErrorStats quietFlat = new ErrorStats();

    private boolean lastA, lastB, lastX, lastUp, lastDown;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, 10);
        shootMotor = robot.shootMotor;
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        // Jam reverses would mess up the intake data
        robot.intakeJamGuard = null;

        double hiveX = 72 + ShotCalculator.HIVE_X_FROM_CENTER;
        double lineY = 72 + WorkingBotButtonMap.STRAFE_LINE_DISTANCE;
        double minX = hiveX - WorkingBotButtonMap.STRAFE_RIGHT;
        double maxX = hiveX + WorkingBotButtonMap.STRAFE_LEFT;
        simX = minX;
        ShotCalculator.Shot first = ShotCalculator.solve(simX, lineY, Math.PI / 2, 0, 0, 0, 0, 0, false);
        simHeading = first.valid ? first.heading : Math.PI / 2;

        telemetry.addLine("Robot stays still. Intake, transfer and flywheel start on play");
        telemetry.addLine("dpad up/down balls, A start trial, B end trial, X strafe/fixed RPM");
        telemetry.update();
        waitForStart();

        Log.i("BallLog", HEADER);
        Log.i("BallEvent", String.format(Locale.US, "start strafe x %.1f..%.1f y %.1f speed %.1f accel %.1f",
                minX, maxX, lineY, SIM_STRAFE_SPEED, SIM_STRAFE_ACCEL));
        file.append(HEADER).append('\n');
        robot.setPowers(new MotorPowers());
        ElapsedTime clock = new ElapsedTime();
        ElapsedTime voltageTimer = new ElapsedTime();
        double volts = robot.getBatteryVoltage();
        robot.setShooterVoltage(volts);
        double lastTime = 0;
        double lastTarget = Double.NaN;
        double rampEndTime = -1;
        double shootAmps = 0, intakeAmps = 0;
        double simMin = Double.POSITIVE_INFINITY, simMax = Double.NEGATIVE_INFINITY;

        while (opModeIsActive()) {
            for (LynxModule hub : robot.hubs) {
                hub.clearBulkCache();
            }
            double t = clock.seconds();
            double dt = Math.max(1e-3, Math.min(0.1, t - lastTime));
            lastTime = t;
            handleButtons(t);

            double target;
            if (strafeMode) {
                target = stepStrafe(dt, minX, maxX, lineY);
                simMin = Math.min(simMin, target);
                simMax = Math.max(simMax, target);
            } else {
                target = FIXED_RPM;
            }
            double slope = Double.isNaN(lastTarget) ? 0 : (target - lastTarget) / dt;
            lastTarget = target;

            robot.setShootRPMRamped(target);
            robot.setIntakePower(INTAKE_POWER);
            robot.setShootFeedPower(TRANSFER_POWER);
            if (voltageTimer.seconds() >= VOLTAGE_READ_SECONDS) {
                volts = robot.getBatteryVoltage();
                robot.setShooterVoltage(volts);
                voltageTimer.reset();
            }

            double rpm = robot.getShootRPM();
            double err = rpm - target;
            boolean ramping = robot.isShooterRamping();
            if (ramping) {
                rampEndTime = -1;
            } else if (rampEndTime < 0) {
                rampEndTime = t;
            }
            boolean settled = rampEndTime >= 0 && t - rampEndTime >= SETTLE_TIME;
            if (READ_CURRENTS) {
                shootAmps = shootMotor.getCurrent(CurrentUnit.AMPS);
                intakeAmps = intakeMotor.getCurrent(CurrentUnit.AMPS);
            }
            shotCounter.update(rpm, target, t, !ramping);
            if (settled) {
                if (inTrial) {
                    trialStats.add(err, rpm);
                } else {
                    quietAll.add(err, rpm);
                    if (slope > SLOPE_RPM_PER_S) {
                        quietRising.add(err, rpm);
                    } else if (slope < -SLOPE_RPM_PER_S) {
                        quietFalling.add(err, rpm);
                    } else {
                        quietFlat.add(err, rpm);
                    }
                }
            }

            row.setLength(0);
            appendFixed(row, t, 3).append(',');
            appendFixed(row, dt * 1000, 1).append(',');
            row.append(trial).append(',').append(balls).append(',').append(inTrial ? 1 : 0).append(',')
                    .append(strafeMode ? 'S' : 'F').append(',');
            appendFixed(row, target, 2).append(',');
            appendFixed(row, rpm, 2).append(',');
            appendFixed(row, err, 2).append(',');
            row.append(ramping ? 1 : 0).append(',').append(shootMotor.getCurrentPosition()).append(',');
            appendFixed(row, shootAmps, 2).append(',');
            appendFixed(row, intakeMotor.getVelocity(), 1).append(',');
            row.append(intakeMotor.getCurrentPosition()).append(',');
            appendFixed(row, intakeAmps, 2).append(',');
            appendFixed(row, volts, 2).append(',');
            row.append(inTrial ? shotCounter.getShots() : 0).append(',');
            appendFixed(row, simX, 2).append(',');
            appendFixed(row, simVx, 2).append(',');
            appendFixed(row, slope, 1);
            Log.i("BallLog", row.toString());
            if (fileRows < MAX_FILE_ROWS) {
                file.append(row).append('\n');
                fileRows++;
            }

            telemetry.addData("Trial", "%s #%d, %d balls (%d of %d done for this count)",
                    inTrial ? "RUNNING" : "waiting", trial, balls, trialsDone[balls], TRIALS_PER_COUNT);
            telemetry.addData("Done", "1:%d 2:%d 3:%d 4:%d", trialsDone[1], trialsDone[2], trialsDone[3], trialsDone[4]);
            telemetry.addData("Flywheel", "%s target %.1f rpm %.1f err %.1f%s", strafeMode ? "strafe" : "fixed",
                    target, rpm, err, ramping ? " RAMPING" : "");
            telemetry.addData("Strafe range", "%.1f..%.1f rpm", simMin, simMax);
            telemetry.addData("Shots counted", "%d", shotCounter.getShots());
            telemetry.addData("Intake", "%.0f tps %.2f A, flywheel %.2f A, %.2f V",
                    intakeMotor.getVelocity(), intakeAmps, shootAmps, volts);
            telemetry.addData("Last trial", lastTrialSummary);
            telemetry.addData("No-ball error", quietAll.describe());
            telemetry.update();
        }

        if (inTrial) {
            endTrial(lastTime);
        }
        robot.setShootRPMRamped(0);
        robot.setIntakePower(0);
        robot.setShootFeedPower(0);
        Log.i("BallSummary", String.format(Locale.US, "strafe target range %.1f..%.1f rpm", simMin, simMax));
        Log.i("BallSummary", "no balls, all: " + quietAll.describe());
        Log.i("BallSummary", "no balls, target rising: " + quietRising.describe());
        Log.i("BallSummary", "no balls, target falling: " + quietFalling.describe());
        Log.i("BallSummary", "no balls, target flat: " + quietFlat.describe());
        Log.i("BallSummary", String.format(Locale.US, "trials done 1:%d 2:%d 3:%d 4:%d",
                trialsDone[1], trialsDone[2], trialsDone[3], trialsDone[4]));
        File logFile = AppUtil.getInstance().getSettingsFile("ball_count_log.csv");
        ReadWriteFile.writeFile(logFile, file.toString());
        Log.i("BallSummary", "saved " + fileRows + " rows to " + logFile.getAbsolutePath());
    }

    // Moves the pretend robot along the strafe line and returns the RPM teleop would ask for there
    private double stepStrafe(double dt, double minX, double maxX, double lineY) {
        double end = WorkingBotButtonMap.END_TOLERANCE;
        if (simDirection > 0 && simX >= maxX - end) {
            simDirection = -1;
        } else if (simDirection < 0 && simX <= minX + end) {
            simDirection = 1;
        }
        double dv = simDirection * SIM_STRAFE_SPEED - simVx;
        double maxDv = SIM_STRAFE_ACCEL * dt;
        dv = Math.max(-maxDv, Math.min(maxDv, dv));
        simAx = dv / dt;
        simVx += dv;
        simX += simVx * dt;
        ShotCalculator.Shot shot = ShotCalculator.solve(simX, lineY, simHeading, simVx, 0, simAx, 0, simTurnRate, false);
        if (!shot.valid) {
            return FIXED_RPM;
        }
        // Robot is assumed to be aimed, so it turns with the aim heading
        double turn = Math.IEEEremainder(shot.heading - simHeading, 2 * Math.PI);
        simTurnRate = turn / dt;
        simHeading = shot.heading;
        return shot.rpm;
    }

    private void handleButtons(double t) {
        boolean a = gamepad1.a, b = gamepad1.b, x = gamepad1.x, up = gamepad1.dpad_up, down = gamepad1.dpad_down;
        if (up && !lastUp && !inTrial) {
            balls = Math.min(4, balls + 1);
        }
        if (down && !lastDown && !inTrial) {
            balls = Math.max(1, balls - 1);
        }
        if (x && !lastX) {
            strafeMode = !strafeMode;
            Log.i("BallEvent", String.format(Locale.US, "t=%.3f mode %s", t, strafeMode ? "strafe" : "fixed"));
        }
        if (a && !lastA) {
            if (inTrial) {
                endTrial(t);
            }
            trial++;
            inTrial = true;
            trialStart = t;
            shotCounter.reset();
            trialStats = new ErrorStats();
            Log.i("BallEvent", String.format(Locale.US, "t=%.3f trial %d start balls=%d", t, trial, balls));
        }
        if (b && !lastB && inTrial) {
            endTrial(t);
        }
        lastA = a;
        lastB = b;
        lastX = x;
        lastUp = up;
        lastDown = down;
    }

    private void endTrial(double t) {
        inTrial = false;
        int counted = shotCounter.getShots();
        trialsDone[balls]++;
        lastTrialSummary = String.format(Locale.US, "#%d %d balls, counted %d, %.1fs", trial, balls, counted, t - trialStart);
        Log.i("BallEvent", String.format(Locale.US, "t=%.3f trial %d end", t, trial));
        Log.i("BallTrial", String.format(Locale.US, "trial=%d balls=%d counted=%d duration=%.2f start=%.3f end=%.3f %s",
                trial, balls, counted, t - trialStart, trialStart, t, trialStats.describe()));
        if (trialsDone[balls] >= TRIALS_PER_COUNT && balls < 4) {
            balls++;
        }
    }

    // Fixed decimal number without String.format, this runs every loop
    private static StringBuilder appendFixed(StringBuilder sb, double value, int decimals) {
        if (!Double.isFinite(value)) {
            return sb.append("nan");
        }
        long scale = 1;
        for (int i = 0; i < decimals; i++) {
            scale *= 10;
        }
        long scaled = Math.round(Math.abs(value) * scale);
        if (value < 0 && scaled != 0) {
            sb.append('-');
        }
        sb.append(scaled / scale);
        if (decimals > 0) {
            sb.append('.');
            String frac = Long.toString(scaled % scale);
            for (int i = frac.length(); i < decimals; i++) {
                sb.append('0');
            }
            sb.append(frac);
        }
        return sb;
    }
}
