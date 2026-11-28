package org.firstinspires.ftc.teamcode.TeleOp;

import android.util.Log;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.ButtonMaps.WorkingBotButtonMap;
import org.firstinspires.ftc.teamcode.MotorPowers;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Shooter.CustomFlywheel;
import org.firstinspires.ftc.teamcode.Shooter.FlywheelVelocityControl;
import org.firstinspires.ftc.teamcode.Shooter.ShotCounter;

import java.io.File;
import java.util.Locale;

// Tests CustomFlywheel (own feedforward + shot recovery) against the hub's PID, robot standing still.
// 1. About 12 s with no balls: holds a few powers and measures KS / KV for FlywheelVelocityControl
// 2. Then runs the strafe target (what teleop asks for while strafing) with intake and transfer on, feed balls in.
// g1: Y = custom / hub PID, X = strafe RPM / fixed RPM. Starts on custom
// Logcat tags: FlyChar (measuring), FlyShot (each shot), FlySummary (end). Every loop goes to
// flywheel_control_log.csv in FIRST/settings on stop
@TeleOp(name = "FlywheelControlTest", group = "Tuning")
public class FlywheelControlTest extends LinearOpMode {
    // ---------------- Tuning ----------------
    // Measure KS / KV first and use them for this run
    public static boolean CHARACTERIZE = true;
    public static double[] CHAR_POWERS = {0.3, 0.4, 0.5, 0.6, 0.7};
    // Each power is held until the RPM stops changing (CHAR_MIN_HOLD to CHAR_MAX_HOLD s), then CHAR_AVERAGE s averaged
    public static double CHAR_MIN_HOLD = 1.5;
    public static double CHAR_MAX_HOLD = 8;
    public static double CHAR_AVERAGE = 0.5;
    public static double CHAR_SETTLED_RPM = 1.5;
    // The KS / KV defaults came from a real run, so the measured ones are only logged unless this is on
    public static boolean APPLY_FIT = false;
    public static double INTAKE_POWER = WorkingBotButtonMap.SHOOT_INTAKE_POWER;
    public static double TRANSFER_POWER = WorkingBotButtonMap.SHOOT_TRANSFER_POWER;
    public static double FIXED_RPM = WorkingBotButtonMap.FIXED_RPM;
    public static double RESEND_SECONDS = 0.1;
    public static double VOLTAGE_READ_SECONDS = 0.25;
    // A shot is followed this long for its dip / recovery / overshoot
    public static double SHOT_WATCH_TIME = 0.8;
    // Back within this of the target = recovered
    public static double RECOVERED_RPM = 4;
    // ----------------------------------------

    private static final String HEADER = "t,dt_ms,ctrl,mode,target,rpm,err,power,ff,integral,boost,spinup,"
            + "shoot_amps,volts,shots,intake_amps,intake_tps,intake_pos";
    private static final int MAX_FILE_ROWS = 60000;

    // Shot and no-ball stats for one controller
    private static class Stats {
        int shots, recovered;
        double dipSum, recSum, overSum, overMax;
        int quietN;
        double quietSumSq, quietMaxAbs;

        String describe() {
            return String.format(Locale.US, "shots=%d avg dip %.1f, recovered %d avg %.3fs, avg overshoot %.1f max %.1f, "
                            + "no-ball rms %.2f max %.1f (n=%d)",
                    shots, shots > 0 ? dipSum / shots : 0, recovered, recovered > 0 ? recSum / recovered : 0,
                    shots > 0 ? overSum / shots : 0, overMax,
                    quietN > 0 ? Math.sqrt(quietSumSq / quietN) : 0, quietMaxAbs, quietN);
        }
    }

    private Robot robot;
    private DcMotorEx shootMotor;
    private DcMotorEx intakeMotor;
    private final StringBuilder file = new StringBuilder();
    private final StringBuilder row = new StringBuilder();
    private int fileRows = 0;
    private final Stats customStats = new Stats();
    private final Stats hubStats = new Stats();

    // Shot being watched
    private double shotTime = -1, shotMinErr, shotRecTime, shotOver;
    private boolean shotCustom;
    private double lastShotTime = Double.NEGATIVE_INFINITY;

    @Override
    public void runOpMode() {
        boolean originalFlag = Robot.USE_CUSTOM_FLYWHEEL;
        robot = new Robot(hardwareMap, 10);
        shootMotor = robot.shootMotor;
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        robot.intakeJamGuard = null;
        ShotCounter shotCounter = new ShotCounter(shootMotor);

        telemetry.addLine("Robot stays still, no balls in it until the measuring is done (about 12 s)");
        telemetry.addLine("Then feed balls. Y custom / hub PID, X strafe / fixed RPM");
        telemetry.update();
        waitForStart();
        robot.setPowers(new MotorPowers());
        robot.setFlowerState(0);

        try {
            if (CHARACTERIZE) {
                characterize();
            }
            if (!opModeIsActive()) {
                return;
            }

            Robot.USE_CUSTOM_FLYWHEEL = true;
            StrafeTargetSim sim = new StrafeTargetSim(FIXED_RPM);
            boolean strafe = true;
            boolean lastX = false, lastY = false;
            file.append(HEADER).append('\n');
            ElapsedTime clock = new ElapsedTime();
            ElapsedTime voltageTimer = new ElapsedTime();
            ElapsedTime resendTimer = new ElapsedTime();
            double volts = robot.getBatteryVoltage();
            robot.setShooterVoltage(volts);
            double lastTime = 0;
            int lastShots = 0;
            double modeStart = 0;

            while (opModeIsActive()) {
                for (LynxModule hub : robot.hubs) {
                    hub.clearBulkCache();
                }
                double t = clock.seconds();
                double dt = Math.max(1e-3, Math.min(0.1, t - lastTime));
                lastTime = t;

                if (gamepad1.y && !lastY) {
                    Robot.USE_CUSTOM_FLYWHEEL = !Robot.USE_CUSTOM_FLYWHEEL;
                    finishShot();
                    shotCounter.reset();
                    lastShots = 0;
                    modeStart = t;
                    Log.i("FlyShot", String.format(Locale.US, "t=%.3f now %s", t, Robot.USE_CUSTOM_FLYWHEEL ? "custom" : "hub PID"));
                }
                if (gamepad1.x && !lastX) {
                    strafe = !strafe;
                }
                lastX = gamepad1.x;
                lastY = gamepad1.y;
                boolean custom = Robot.USE_CUSTOM_FLYWHEEL;

                double target = strafe ? sim.step(dt) : FIXED_RPM;
                if (resendTimer.seconds() >= RESEND_SECONDS) {
                    robot.resendShooterOutputs();
                    resendTimer.reset();
                }
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
                boolean spinningUp = robot.isShooterRamping();
                // Nothing counts for a second after switching controllers, the handover can wobble the RPM
                shotCounter.update(rpm, target, t, !spinningUp && t - modeStart > 1.0);
                int shots = shotCounter.getShots();
                if (shots > lastShots) {
                    finishShot();
                    shotTime = t;
                    shotCustom = custom;
                    shotMinErr = err;
                    shotRecTime = -1;
                    shotOver = 0;
                    lastShots = shots;
                }
                watchShot(t, err);
                // No-ball tracking: spun up, settled a while after switching, no shot lately
                if (!spinningUp && t - modeStart > 2 && t - lastShotTime > SHOT_WATCH_TIME + 0.3 && shotTime < 0) {
                    Stats s = custom ? customStats : hubStats;
                    s.quietN++;
                    s.quietSumSq += err * err;
                    s.quietMaxAbs = Math.max(s.quietMaxAbs, Math.abs(err));
                }

                double shootAmps = shootMotor.getCurrent(CurrentUnit.AMPS);
                double intakeAmps = intakeMotor.getCurrent(CurrentUnit.AMPS);
                CustomFlywheel cf = robot.getCustomFlywheel();
                FlywheelVelocityControl control = cf != null ? cf.getControl() : null;
                boolean active = custom && control != null;

                row.setLength(0);
                appendFixed(row, t, 3).append(',');
                appendFixed(row, dt * 1000, 1).append(',');
                row.append(custom ? 'C' : 'H').append(',').append(strafe ? 'S' : 'F').append(',');
                appendFixed(row, target, 2).append(',');
                appendFixed(row, rpm, 2).append(',');
                appendFixed(row, err, 2).append(',');
                appendFixed(row, active ? control.getLastPower() : Double.NaN, 3).append(',');
                appendFixed(row, active ? control.getLastFeedforward() : Double.NaN, 3).append(',');
                appendFixed(row, active ? control.getIntegral() : Double.NaN, 3).append(',');
                // 1 = full power, -1 = braking
                row.append(active && control.isBoosting() ? 1 : active && control.isBraking() ? -1 : 0)
                        .append(',').append(spinningUp ? 1 : 0).append(',');
                appendFixed(row, shootAmps, 2).append(',');
                appendFixed(row, volts, 2).append(',');
                row.append(shots).append(',');
                appendFixed(row, intakeAmps, 2).append(',');
                // Intake encoder, to see if balls going in can be counted from the intake slowing down
                appendFixed(row, intakeMotor.getVelocity(), 1).append(',');
                row.append(intakeMotor.getCurrentPosition());
                if (fileRows < MAX_FILE_ROWS) {
                    file.append(row).append('\n');
                    fileRows++;
                }

                telemetry.addData("Controller", "%s (Y to switch), %s target (X)", custom ? "CUSTOM" : "HUB PID",
                        strafe ? "strafe" : "fixed");
                telemetry.addData("Flywheel", "target %.1f rpm %.1f err %.1f%s", target, rpm, err,
                        spinningUp ? " SPINNING UP" : "");
                if (active) {
                    telemetry.addData("Power", "%.3f ff %.3f I %.3f%s", control.getLastPower(),
                            control.getLastFeedforward(), control.getIntegral(), control.isBoosting() ? " FULL" : "");
                }
                telemetry.addData("KS / KV", "%.3f V / %.5f V per rpm", FlywheelVelocityControl.KS, FlywheelVelocityControl.KV);
                telemetry.addData("Shots", "%d this mode", shots);
                telemetry.addData("Intake encoder", "%d ticks, %.0f tps%s", intakeMotor.getCurrentPosition(),
                        intakeMotor.getVelocity(), intakeMotor.getCurrentPosition() == 0 ? " (NOT READING)" : "");
                telemetry.addData("Custom", customStats.describe());
                telemetry.addData("Hub PID", hubStats.describe());
                telemetry.update();
            }
            finishShot();
            Log.i("FlySummary", String.format(Locale.US, "KS %.4f KV %.6f KA %.4f KP %.4f KI %.4f boost %.1f/%.1f",
                    FlywheelVelocityControl.KS, FlywheelVelocityControl.KV, FlywheelVelocityControl.KA,
                    FlywheelVelocityControl.KP, FlywheelVelocityControl.KI,
                    FlywheelVelocityControl.BOOST_ENTER_RPM, FlywheelVelocityControl.BOOST_EXIT_RPM));
            Log.i("FlySummary", String.format(Locale.US, "strafe target range %.1f..%.1f", sim.getMinTarget(), sim.getMaxTarget()));
            Log.i("FlySummary", "custom: " + customStats.describe());
            Log.i("FlySummary", "hub PID: " + hubStats.describe());
        } finally {
            robot.setShootRPMRamped(0);
            Robot.USE_CUSTOM_FLYWHEEL = originalFlag;
            robot.setShootRPMRamped(0);
            robot.setIntakePower(0);
            robot.setShootFeedPower(0);
            if (fileRows > 0) {
                File logFile = AppUtil.getInstance().getSettingsFile("flywheel_control_log.csv");
                ReadWriteFile.writeFile(logFile, file.toString());
                Log.i("FlySummary", "saved " + fileRows + " rows to " + logFile.getAbsolutePath());
            }
        }
    }

    private void watchShot(double t, double err) {
        if (shotTime < 0) {
            return;
        }
        if (t - shotTime > SHOT_WATCH_TIME) {
            finishShot();
            return;
        }
        shotMinErr = Math.min(shotMinErr, err);
        if (shotRecTime < 0) {
            if (t - shotTime > 0.03 && err > -RECOVERED_RPM) {
                shotRecTime = t - shotTime;
            }
        } else {
            shotOver = Math.max(shotOver, err);
        }
    }

    private void finishShot() {
        if (shotTime < 0) {
            return;
        }
        Stats s = shotCustom ? customStats : hubStats;
        s.shots++;
        s.dipSum += shotMinErr;
        if (shotRecTime >= 0) {
            s.recovered++;
            s.recSum += shotRecTime;
        }
        s.overSum += shotOver;
        s.overMax = Math.max(s.overMax, shotOver);
        Log.i("FlyShot", String.format(Locale.US, "t=%.3f %s dip %.1f recovered %s overshoot %.1f",
                shotTime, shotCustom ? "custom" : "hub", shotMinErr,
                shotRecTime >= 0 ? String.format(Locale.US, "%.3fs", shotRecTime) : "no", shotOver));
        lastShotTime = shotTime;
        shotTime = -1;
    }

    // Holds each power, averages RPM and volts, fits volts = KS + KV * rpm
    private void characterize() {
        shootMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int n = CHAR_POWERS.length;
        double[] rpms = new double[n];
        double[] voltsApplied = new double[n];
        for (int i = 0; i < n && opModeIsActive(); i++) {
            double power = CHAR_POWERS[i];
            shootMotor.setPower(power);
            ElapsedTime timer = new ElapsedTime();
            double rpmSum = 0, voltSum = 0;
            int count = 0;
            // Waits until it stops speeding up: the average of each CHAR_AVERAGE window is within CHAR_SETTLED_RPM
            // of the window before. Last time 2.2 s wasn't enough and KS came out 1 V too high
            double windowStart = 0, windowSum = 0, lastWindowMean = Double.NaN;
            int windowCount = 0;
            boolean settled = false;
            while (opModeIsActive() && timer.seconds() < CHAR_MAX_HOLD) {
                for (LynxModule hub : robot.hubs) {
                    hub.clearBulkCache();
                }
                double t = timer.seconds();
                double rpm = robot.getShootRPM();
                if (settled) {
                    rpmSum += rpm;
                    voltSum += robot.getBatteryVoltage();
                    count++;
                    if (count >= 20 && t - windowStart >= CHAR_AVERAGE) {
                        break;
                    }
                } else {
                    windowSum += rpm;
                    windowCount++;
                    if (t - windowStart >= CHAR_AVERAGE) {
                        double mean = windowSum / windowCount;
                        if (t >= CHAR_MIN_HOLD && Math.abs(mean - lastWindowMean) < CHAR_SETTLED_RPM) {
                            settled = true;
                        }
                        lastWindowMean = mean;
                        windowSum = 0;
                        windowCount = 0;
                        windowStart = t;
                    }
                }
                telemetry.addData("Measuring", "power %.2f (%d of %d), %.1f rpm%s", power, i + 1, n, rpm,
                        settled ? ", settled" : "");
                telemetry.update();
            }
            if (!settled) {
                Log.w("FlyChar", String.format(Locale.US, "power %.2f never settled", power));
            }
            if (count > 0) {
                rpms[i] = rpmSum / count;
                voltsApplied[i] = power * voltSum / count;
                Log.i("FlyChar", String.format(Locale.US, "power %.2f rpm %.2f battery %.2f V applied %.3f V",
                        power, rpms[i], voltSum / count, voltsApplied[i]));
            }
        }
        // Least squares line through (rpm, volts)
        double sx = 0, sy = 0, sxx = 0, sxy = 0;
        for (int i = 0; i < n; i++) {
            sx += rpms[i];
            sy += voltsApplied[i];
            sxx += rpms[i] * rpms[i];
            sxy += rpms[i] * voltsApplied[i];
        }
        double denom = n * sxx - sx * sx;
        if (denom > 1e-6) {
            double kv = (n * sxy - sx * sy) / denom;
            double ks = (sy - kv * sx) / n;
            // Only used if it looks like a real motor, otherwise the defaults stay
            boolean sane = kv > 0.01 && kv < 0.06 && ks > -0.5 && ks < 2.5;
            Log.i("FlyChar", String.format(Locale.US, "fit KS %.4f KV %.6f %s", ks, kv,
                    !sane ? "NOT used, looks wrong" : APPLY_FIT ? "used" : "logged only (APPLY_FIT off)"));
            if (sane && APPLY_FIT) {
                FlywheelVelocityControl.KS = Math.max(0, ks);
                FlywheelVelocityControl.KV = kv;
            }
        }
        shootMotor.setPower(0);
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
