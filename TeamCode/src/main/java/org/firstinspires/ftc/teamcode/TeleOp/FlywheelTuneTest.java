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
import org.firstinspires.ftc.teamcode.ButtonMaps.WorkingBotButtonMap;
import org.firstinspires.ftc.teamcode.MotorPowers;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Shooter.FlywheelVelocityControl;
import org.firstinspires.ftc.teamcode.Shooter.ShotCounter;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

// Tries a list of CustomFlywheel settings, one change at a time from the defaults, robot standing still.
// For each setting the screen says what to do. Press A, then:
//   1. Step test, no balls (~14 s): hold, step up to STEP_HIGH_RPM and back, strafe-like ramp up and down
//   2. Screen says FEED: feed BALLS_PER_TEST balls. It moves on once it counted them (or after FEED_TIMEOUT)
// The defaults run first, in the middle and last, to see battery drift. Nothing to tune by hand.
// Logcat tag FlyTune: one result line per setting. Every loop goes to flywheel_tune_log.csv, saved after each test.
// Saving it all on stop took too long (1.9 MB) and the robot app restarted with "stuck in stop()"
@TeleOp(name = "FlywheelTuneTest", group = "Tuning")
public class FlywheelTuneTest extends LinearOpMode {
    // ---------------- Tuning ----------------
    public static double HOLD_RPM = WorkingBotButtonMap.FIXED_RPM;
    // Step up target, about a long shot
    public static double STEP_HIGH_RPM = 250;
    // Strafe-like ramp: how far and how fast the target moves, RPM and RPM/s
    public static double RAMP_RPM = 30;
    public static double RAMP_RATE = 20;
    public static int BALLS_PER_TEST = 3;
    public static double FEED_TIMEOUT = 20;
    // Back within this of the target = there
    public static double SETTLED_RPM = 4;
    public static double SHOT_WATCH_TIME = 0.8;
    public static double INTAKE_POWER = WorkingBotButtonMap.SHOOT_INTAKE_POWER;
    public static double TRANSFER_POWER = WorkingBotButtonMap.SHOOT_TRANSFER_POWER;
    public static double VOLTAGE_READ_SECONDS = 0.25;
    // ----------------------------------------

    // Settings the list can change
    private static final String[] NAMES = {"KP", "KI", "KA", "BOOST_ENTER", "BOOST_EXIT", "BRAKE_ENTER", "BRAKE_POWER"};
    // Each test: setting index (-1 = all defaults) and its value
    private static final double[][] TESTS = {
            {-1, 0},
            {0, 0.004}, {0, 0.014},
            {1, 0}, {1, 0.05},
            {4, 2}, {4, 8},
            {-1, 0},
            {3, 7}, {3, 14},
            {2, 0}, {2, 0.05},
            {6, 0}, {6, -0.3},
            {5, 4}, {5, 9},
            {-1, 0},
    };

    private static double get(int i) {
        switch (i) {
            case 0: return FlywheelVelocityControl.KP;
            case 1: return FlywheelVelocityControl.KI;
            case 2: return FlywheelVelocityControl.KA;
            case 3: return FlywheelVelocityControl.BOOST_ENTER_RPM;
            case 4: return FlywheelVelocityControl.BOOST_EXIT_RPM;
            case 5: return FlywheelVelocityControl.BRAKE_ENTER_RPM;
            default: return FlywheelVelocityControl.BRAKE_POWER;
        }
    }

    private static void set(int i, double v) {
        switch (i) {
            case 0: FlywheelVelocityControl.KP = v; break;
            case 1: FlywheelVelocityControl.KI = v; break;
            case 2: FlywheelVelocityControl.KA = v; break;
            case 3: FlywheelVelocityControl.BOOST_ENTER_RPM = v; break;
            case 4: FlywheelVelocityControl.BOOST_EXIT_RPM = v; break;
            case 5: FlywheelVelocityControl.BRAKE_ENTER_RPM = v; break;
            default: FlywheelVelocityControl.BRAKE_POWER = v; break;
        }
    }

    private static String label(double[] test) {
        int i = (int) test[0];
        return i < 0 ? "defaults" : String.format(Locale.US, "%s=%.4g", NAMES[i], test[1]);
    }

    // Step test phases
    private static final int HOLD = 0, UP = 1, DOWN = 2, RAMP_UP = 3, RAMP_DOWN = 4, FEED = 5;
    private static final double HOLD_TIME = 3, UP_TIME = 3, DOWN_TIME = 3;

    private Robot robot;
    private DcMotorEx shootMotor;
    private final StringBuilder file = new StringBuilder();
    private int fileRows = 0;
    private static final int MAX_FILE_ROWS = 120000;
    private double volts = 12;
    private final ElapsedTime voltageTimer = new ElapsedTime();
    private File logFile = null;

    // Adds the rows kept since the last save to the file
    private void saveRows() {
        if (logFile == null || file.length() == 0) {
            return;
        }
        try (FileWriter writer = new FileWriter(logFile, true)) {
            writer.append(file);
        } catch (IOException e) {
            Log.e("FlyTune", "couldn't save rows", e);
        }
        file.setLength(0);
    }

    // Scores for the test running now
    private double upReached, upOver, downReached, downUnder, holdSumSq, rampSumSq;
    private int holdN, rampN;
    private double shotTime, shotMinErr, shotRecTime, shotOver;
    private int shots, recovered;
    private double dipSum, recSum, overSum;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, 10);
        shootMotor = robot.shootMotor;
        robot.intakeJamGuard = null;
        boolean originalFlag = Robot.USE_CUSTOM_FLYWHEEL;
        double[] defaults = new double[NAMES.length];
        for (int i = 0; i < NAMES.length; i++) {
            defaults[i] = get(i);
        }

        telemetry.addLine("Flywheel tune: " + TESTS.length + " tests, about 30 s each with " + BALLS_PER_TEST + " balls");
        telemetry.addLine("Have " + BALLS_PER_TEST + " balls ready for each test. Press start");
        telemetry.update();
        waitForStart();
        robot.setPowers(new MotorPowers());
        robot.setFlowerState(0);
        Robot.USE_CUSTOM_FLYWHEEL = true;
        logFile = AppUtil.getInstance().getSettingsFile("flywheel_tune_log.csv");
        ReadWriteFile.writeFile(logFile, "t,test,label,phase,target,rpm,err,power,boost,shoot_amps,volts,shots\n");
        volts = robot.getBatteryVoltage();
        robot.setShooterVoltage(volts);
        ElapsedTime clock = new ElapsedTime();

        try {
            for (int n = 0; n < TESTS.length && opModeIsActive(); n++) {
                for (int i = 0; i < NAMES.length; i++) {
                    set(i, defaults[i]);
                }
                if (TESTS[n][0] >= 0) {
                    set((int) TESTS[n][0], TESTS[n][1]);
                }
                String label = label(TESTS[n]);

                // Wait for A, flywheel holds speed meanwhile
                boolean released = !gamepad1.a;
                while (opModeIsActive()) {
                    hold(HOLD_RPM, false);
                    telemetry.addData("NEXT", "test %d of %d: %s", n + 1, TESTS.length, label);
                    telemetry.addLine("1. Make sure no balls are in the robot");
                    telemetry.addLine("2. Press A. The flywheel runs by itself for about 14 s");
                    telemetry.addLine("3. When the screen says FEED, feed " + BALLS_PER_TEST + " balls");
                    telemetry.update();
                    if (!gamepad1.a) {
                        released = true;
                    } else if (released) {
                        break;
                    }
                }
                runTest(n, label, clock);
                saveRows();
            }
            telemetry.addLine("ALL DONE. Stop the op mode, then plug in USB");
            telemetry.update();
            Log.i("FlyTune", "all tests done");
            while (opModeIsActive()) {
                hold(0, false);
                idle();
            }
        } finally {
            for (int i = 0; i < NAMES.length; i++) {
                set(i, defaults[i]);
            }
            robot.setShootRPMRamped(0);
            Robot.USE_CUSTOM_FLYWHEEL = originalFlag;
            robot.setShootRPMRamped(0);
            robot.setIntakePower(0);
            robot.setShootFeedPower(0);
            // Only the rows of a test that was cut short are left, so this is quick
            saveRows();
            Log.i("FlyTune", "saved " + fileRows + " rows");
        }
    }

    // One loop at a target, intake / transfer on or off, returns the RPM
    private double hold(double target, boolean feed) {
        for (LynxModule hub : robot.hubs) {
            hub.clearBulkCache();
        }
        robot.setShootRPMRamped(target);
        robot.setIntakePower(feed ? INTAKE_POWER : 0);
        robot.setShootFeedPower(feed ? TRANSFER_POWER : 0);
        if (voltageTimer.seconds() >= VOLTAGE_READ_SECONDS) {
            volts = robot.getBatteryVoltage();
            robot.setShooterVoltage(volts);
            voltageTimer.reset();
        }
        return robot.getShootRPM();
    }

    private void runTest(int n, String label, ElapsedTime clock) {
        upReached = -1;
        upOver = 0;
        downReached = -1;
        downUnder = 0;
        holdSumSq = 0;
        holdN = 0;
        rampSumSq = 0;
        rampN = 0;
        shots = 0;
        recovered = 0;
        dipSum = 0;
        recSum = 0;
        overSum = 0;
        shotTime = -1;
        ShotCounter shotCounter = new ShotCounter(shootMotor);
        double rampTime = RAMP_RPM / Math.max(1, RAMP_RATE);
        double[] lengths = {HOLD_TIME, UP_TIME, DOWN_TIME, rampTime, rampTime, FEED_TIMEOUT};
        int phase = HOLD;
        double phaseStart = clock.seconds();
        int lastShots = 0;
        Log.i("FlyTune", String.format(Locale.US, "test %d %s start", n + 1, label));

        while (opModeIsActive() && phase <= FEED) {
            double t = clock.seconds();
            double dt = t - phaseStart;
            if (dt >= lengths[phase] || (phase == FEED && shotCounter.getShots() >= BALLS_PER_TEST && shotTime < 0)) {
                phase++;
                phaseStart = t;
                dt = 0;
                if (phase > FEED) {
                    break;
                }
            }
            double target;
            switch (phase) {
                case UP: target = STEP_HIGH_RPM; break;
                case RAMP_UP: target = HOLD_RPM + RAMP_RATE * dt; break;
                case RAMP_DOWN: target = HOLD_RPM + RAMP_RPM - RAMP_RATE * dt; break;
                default: target = HOLD_RPM; break;
            }
            boolean feed = phase == FEED;
            double rpm = hold(target, feed);
            double err = rpm - target;

            switch (phase) {
                case HOLD:
                    // The first second is settling from before
                    if (dt > 1) {
                        holdSumSq += err * err;
                        holdN++;
                    }
                    break;
                case UP:
                    if (upReached < 0) {
                        if (err > -SETTLED_RPM) {
                            upReached = dt;
                        }
                    } else {
                        upOver = Math.max(upOver, err);
                    }
                    break;
                case DOWN:
                    if (downReached < 0) {
                        if (err < SETTLED_RPM) {
                            downReached = dt;
                        }
                    } else {
                        downUnder = Math.min(downUnder, err);
                    }
                    break;
                case RAMP_UP:
                case RAMP_DOWN:
                    // Skips the corners where the ramp changes direction
                    if (dt > 0.3) {
                        rampSumSq += err * err;
                        rampN++;
                    }
                    break;
                default:
                    // Feeding: counts and scores each shot
                    shotCounter.update(rpm, target, t, !robot.isShooterRamping() && dt > 0.5);
                    int s = shotCounter.getShots();
                    if (s > lastShots) {
                        finishShot();
                        shotTime = t;
                        shotMinErr = err;
                        shotRecTime = -1;
                        shotOver = 0;
                    }
                    lastShots = s;
                    watchShot(t, err);
                    break;
            }

            FlywheelVelocityControl control = robot.getCustomFlywheel() != null
                    ? robot.getCustomFlywheel().getControl() : null;
            double power = control != null ? control.getLastPower() : Double.NaN;
            if (fileRows < MAX_FILE_ROWS) {
                file.append(String.format(Locale.US, "%.3f,%d,%s,%d,%.2f,%.2f,%.2f,%.3f,%d,%.2f,%.2f,%d\n",
                        t, n + 1, label, phase, target, rpm, err, power,
                        control != null && control.isBoosting() ? 1 : control != null && control.isBraking() ? -1 : 0,
                        shootMotor.getCurrent(CurrentUnit.AMPS), volts, shotCounter.getShots()));
                fileRows++;
            }

            telemetry.addData("Test", "%d of %d: %s", n + 1, TESTS.length, label);
            if (feed) {
                telemetry.addLine(String.format(Locale.US, ">>> FEED %d BALLS NOW <<<  (%d counted)",
                        BALLS_PER_TEST, shotCounter.getShots()));
            } else {
                telemetry.addLine("Running by itself, no balls yet");
            }
            telemetry.addData("Flywheel", "target %.1f rpm %.1f err %.1f power %.2f", target, rpm, err, power);
            telemetry.update();
        }
        finishShot();
        String result = String.format(Locale.US,
                "test %d %s: up %s over %.1f, down %s under %.1f, hold rms %.2f, ramp rms %.2f, "
                        + "shots %d avg dip %.1f back in %s overshoot %.1f, %.2f V",
                n + 1, label,
                upReached >= 0 ? String.format(Locale.US, "%.2fs", upReached) : "never", upOver,
                downReached >= 0 ? String.format(Locale.US, "%.2fs", downReached) : "never", downUnder,
                holdN > 0 ? Math.sqrt(holdSumSq / holdN) : 0, rampN > 0 ? Math.sqrt(rampSumSq / rampN) : 0,
                shots, shots > 0 ? dipSum / shots : 0,
                recovered > 0 ? String.format(Locale.US, "%.3fs", recSum / recovered) : "-",
                shots > 0 ? overSum / shots : 0, volts);
        Log.i("FlyTune", result);
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
            if (t - shotTime > 0.03 && err > -SETTLED_RPM) {
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
        shots++;
        dipSum += shotMinErr;
        if (shotRecTime >= 0) {
            recovered++;
            recSum += shotRecTime;
        }
        overSum += shotOver;
        shotTime = -1;
    }
}
