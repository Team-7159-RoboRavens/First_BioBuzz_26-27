package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.ButtonMaps.WorkingBotButtonMap;
import org.firstinspires.ftc.teamcode.MotorPowers;
import org.firstinspires.ftc.teamcode.Robot;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import java.io.File;
import java.util.Formatter;
import java.util.List;
import java.util.Locale;

// TeleOpWorkingBotBlue / TeleOpWorkingBotRed pick the alliance
public abstract class TeleOpWorkingBot extends OpMode{

    VoltageSensor controlHubVoltageSensor;
    public Robot robot;
    WorkingBotButtonMap bm;
    private ElapsedTime lastFrameTime = new ElapsedTime();

    public double averageFrameTime = 0;
    private long frameCount;
    // Limelight ll;

    MotorPowers mp;
    double[] vals;

    // Log: a row every LOG_EVERY loops (about 10 per second), saved to teleop_log.csv when the op mode stops
    private static final int LOG_EVERY = 5;
    private static final int MAX_LOG_ROWS = 6000;
    private final StringBuilder log = new StringBuilder();
    // Writes rows straight into log, no new Formatter or String per row
    private final Formatter logFormatter = new Formatter(log, Locale.US);
    private final ElapsedTime matchTime = new ElapsedTime();
    private int logRows = 0;
    private double lastVoltage = 0;
    private double maxLoopMs = 0;
    // The battery read is its own hub command and only shows on telemetry / the log, so it is read twice a second
    private static final double VOLTAGE_READ_SECONDS = 0.5;
    private final ElapsedTime voltageTimer = new ElapsedTime();
    private boolean haveVoltage = false;

    // Which HIVE to aim at and which wall is the park wall
    protected abstract boolean isRed();

    // ShootTest uses its own button map
    protected WorkingBotButtonMap createButtonMap() {
        return new WorkingBotButtonMap(this, robot, isRed());
    }

    @Override
    public void init() {
        robot = new Robot(hardwareMap, 10);
        bm = createButtonMap();
        controlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        // Heading hold, see InputHandling
        robot.inputHandlingEnabled = true;
    }

    @Override
    public void loop() {
        // Index loop, a for-each makes a new iterator every loop
        List<LynxModule> hubs = robot.hubs;
        for (int i = 0; i < hubs.size(); i++) {
            hubs.get(i).clearBulkCache();
        }
        mp = robot.tuneMotorPowers(bm.getPowers());
        robot.setPowers(mp);
        vals = bm.getState();
        robot.setShootRPMRamped(vals[0]);
        robot.setIntakePower(vals[1]);
        robot.setShootFeedPower(vals[2]);
        robot.setFlowerState(vals[3]);

        double loopMs = lastFrameTime.milliseconds();
        averageFrameTime = averageFrameTime * frameCount + loopMs;
        averageFrameTime /= frameCount + 1;
        lastFrameTime.reset();
        frameCount ++;
        if (loopMs > maxLoopMs) maxLoopMs = loopMs;
        // Bulk cached, read once for the log and telemetry
        double shootRPM = robot.getShootRPM();
        if (frameCount % LOG_EVERY == 0) {
            logSample(shootRPM);
        }
        telemetry.addData("Voltage", "%.2f V, flywheel scale %.3f", lastVoltage, robot.getShooterVoltageScale());
        telemetry.addData("Loop", "avg %.1f ms, max %.1f ms", averageFrameTime, maxLoopMs);
        TeleOpData.frameTime = averageFrameTime;
        telemetry.addData("Motors", "shoot motor target(%.2f), actual(%.2f)",vals[0], shootRPM);
        telemetry.addData("Motors", "left front(%.2f), right front(%.2f)", mp.lf, mp.rf);
        telemetry.addData("Motors", "left back(%.2f), right back(%.2f)", mp.lb, mp.rb);
        telemetry.addData("Heading hold", "enabled(%b) locked(%b)", robot.inputHandlingEnabled, robot.isHeadingLocked());
        if (robot.intakeJamGuard != null) {
            telemetry.addData("Intake", "%.1f A, jams cleared %d%s%s", robot.intakeJamGuard.getCurrent(),
                    robot.intakeJamGuard.getJams(), robot.intakeJamGuard.isReversing() ? ", REVERSING" : "",
                    robot.intakeJamGuard.gaveUp() ? ", gave up (full?)" : "");
        }
    }

    // One CSV row, kept in memory, the file is only written in stop()
    private void logSample(double shootRPM) {
        if (!haveVoltage || voltageTimer.seconds() >= VOLTAGE_READ_SECONDS) {
            lastVoltage = controlHubVoltageSensor.getVoltage();
            voltageTimer.reset();
            haveVoltage = true;
            robot.setShooterVoltage(lastVoltage);
        }
        if (logRows >= MAX_LOG_ROWS) {
            return;
        }
        if (logRows == 0) {
            log.append("time_s,voltage,loop_ms_max,lf,rf,lb,rb,shoot_target_rpm,shoot_rpm,intake,intake_amps,heading_locked,balls\n");
            matchTime.reset();
        }
        logFormatter.format("%.2f,%.2f,%.1f,%.2f,%.2f,%.2f,%.2f,%.0f,%.0f,%.2f,%.1f,%d,%d\n",
                matchTime.seconds(), lastVoltage, maxLoopSinceLog(), mp.lf, mp.rf, mp.lb, mp.rb,
                vals[0], shootRPM, vals[1],
                robot.intakeJamGuard != null ? robot.intakeJamGuard.getCurrent() : 0,
                robot.isHeadingLocked() ? 1 : 0, bm.getBallCount());
        logRows++;
    }

    // Slowest loop since the last row, shows lag spikes the average hides
    private double maxLoopSinceLog() {
        double m = maxLoopMs;
        maxLoopMs = 0;
        return m;
    }

    @Override
    public void stop() {
        if (logRows > 0) {
            File logFile = AppUtil.getInstance().getSettingsFile("teleop_log.csv");
            ReadWriteFile.writeFile(logFile, log.toString());
        }
    }
}
