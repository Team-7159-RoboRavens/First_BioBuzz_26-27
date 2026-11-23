package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.pedropathing.utils.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Autonomous.Essentials.Constants;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOpData;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import android.util.Log;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.function.Supplier;

public abstract class ThreeCycleAutoBase extends OpMode {
    protected Follower follower;
    protected AutoMechanisms mechanisms;
    protected Timer pathTimer, opModeTimer;

    public enum PathState {
        SHOOT_PRELOAD1,
        SHOOT_POS1_SHOOT_POS_1_1,
        SHOOT_POS1_1_INTAKE_POS,
        INTAKE1,
        INTAKE_POS_SHOOT_POS2,
        SHOOT_PRELOAD2,
        PRE_SHOOT_POS2_SHOOT_POS2,
        INTAKE2,
        SHOOT_PRELOAD3,
        SHOOT_POS2_PARK_POS,
        DONE,
        // ExtraLoad autos only
        EXTRA_APPROACH,
        EXTRA_INTAKE,
        EXTRA_PICKUP,

        MOVE
    }

    // Shooting timings (seconds)
    // Only the first volley waits for spin up, the flywheel then stays on for the whole auto
    // Fixed wait from auto start before the first volley feeds. The spin up ramp (Shooter/FlywheelSpinUp) keeps it
    // from overshooting, so lower this until the first ball starts falling short
    protected static final double SPIN_UP_TIME = 4.7;
    // How long each volley feeds
    protected static final double FEED_TIME_1 = 4.4;
    protected static final double FEED_TIME_2 = 3.8;
    protected static final double FEED_TIME_3 = 5.2;

    // Flywheel RPM for the preload volley and the later volleys
    protected static final double FIRST_SHOT_RPM = 192;
    protected static final double SHOT_RPM = 192;

    // Wait this long before trusting follower.isBusy() after starting a path
    protected static final double MIN_PATH_TIME = 0.1;

    // Max seconds per path before moving on (only there so a stuck path can't hang the auto)
    protected static final double SHOOT1_TO_SHOOT1_1_TIMEOUT = 3.0;
    protected static final double SHOOT1_1_TO_INTAKE_TIMEOUT = 5.0;
    protected static final double INTAKE_TO_SHOOT2_TIMEOUT = 8.0;
    protected static final double PRE_SHOOT2_TO_SHOOT2_TIMEOUT = 2.0;
    protected static final double SHOOT2_TO_PARK_TIMEOUT = 6.0;

    // Skip to park after this many seconds
    protected static final double PARK_DEADLINE = 27.3;

    // Seconds to sit at the flower with the intake running, raise if balls are left behind
    protected static final double INTAKE1_TIME = 0.5;
    // How close to its end the intake path has to get before it counts as done (Pedro default is 0.025)
    protected static final double INTAKE_END_T = 0.01;

    // Face the direction of travel in the middle of the intake to shoot 2 path (false = old linear heading)
    protected static final boolean FACE_TRAVEL_TO_SHOOT2 = true;
    protected static final double TURN_IN_END = 0.25;
    protected static final double TURN_OUT_START = 0.7;

    protected PathState pathState;

    // Path for this state already handed to the follower
    private boolean pathStarted = false;
    // Volley state, reset on every state change: when feeding started
    private double feedStartTime = -1;
    private double lastLogTime = 0;

    // Numbers for each finished path, shown on telemetry
    private final List<String> pathReports = new ArrayList<>();
    private double statLastTime, statDistance, statMaxSpeed, statTurnTime, statOffPathTime, statBrakeTime;

    // Abstract pose getters to be implemented by Red/Blue subclasses
    protected abstract Pose getShootPose1();
    protected abstract Pose getShootPose1_0();
    protected abstract Pose getShootPose1_1();
    protected abstract Pose getIntakePose();
    protected abstract Pose getIntakeControl();
    protected abstract Pose getPreShootPose2();
    protected abstract Pose getShootPose2();
    protected abstract Pose getShoot2Control();
    protected abstract Pose getShoot2Control2();
    protected abstract Pose getParkPose();
    protected abstract Pose getParkControl();

    // Paths are built from the robot's actual position when their state starts
    // Heading comes from the planned pose so it never wraps the long way

    protected Pose fromActual(Pose nominal) {
        Pose p = follower.pose();
        return new Pose(p.x(), p.y(), nominal.heading());
    }

    protected Path lineTo(Pose nominalStart, Pose end) {
        Pose start = fromActual(nominalStart);
        return line(start, end).linear(start, end);
    }

    protected Path curveTo(Pose nominalStart, Pose control, Pose end) {
        Pose start = fromActual(nominalStart);
        return curve(start, control, end).linear(start, end);
    }
    // Has to get closer to the flower than a normal path before it counts as done
    protected Path buildIntakePath() {
        return curveTo(getShootPose1_1(), getIntakeControl(), getIntakePose())
                .with(Constants.foresightConfig.parametricTConstraint.at(INTAKE_END_T));
    }

    // Mecanum is fastest driving forward, so turn to face the travel direction, then turn to the shoot heading
    protected Path buildShoot2Path() {
        Pose start = fromActual(getIntakePose());
        Pose end = getPreShootPose2();
        Path path = curve(start, getShoot2Control(), end);
        if (!FACE_TRAVEL_TO_SHOOT2) {
            return path.linear(start, end);
        }
        double turnInHeading = path.curve.tangent(path.curve.parameter(TURN_IN_END)).theta();
        double turnOutHeading = path.curve.tangent(path.curve.parameter(TURN_OUT_START)).theta();
        return path.heading(Interpolator.piecewise()
                .until(TURN_IN_END, Interpolator.linear(start.heading(), turnInHeading))
                .until(TURN_OUT_START, Interpolator.tangent)
                .until(1.0, Interpolator.linear(turnOutHeading, end.heading())));
    }

    // ExtraLoad overrides this
    protected Path buildParkPath() {
        return curveTo(getShootPose2(), getParkControl(), getParkPose());
    }

    // Starts the path once, then returns true when it is done or timed out
    protected boolean runPath(Supplier<Path> pathSupplier, double timeout) {
        if (!pathStarted) {
            follower.follow(pathSupplier.get());
            pathStarted = true;
            statLastTime = pathTimer.seconds();
            statDistance = statMaxSpeed = statTurnTime = statOffPathTime = statBrakeTime = 0;
            Log.i(pathState.toString(), "path started");
            return false;
        }
        double t = pathTimer.seconds();
        updatePathStats(t);
        boolean timedOut = t > timeout;
        boolean done = (t > MIN_PATH_TIME && !follower.isBusy()) || timedOut;
        if (done) reportPath(t, timedOut);
        return done;
    }

    private void updatePathStats(double t) {
        double dt = t - statLastTime;
        statLastTime = t;
        double speed = follower.velocity().linSpeed();
        statDistance += speed * dt;
        statMaxSpeed = Math.max(statMaxSpeed, speed);
        if (follower.following() && follower.algorithm() instanceof Foresight) {
            Foresight foresight = (Foresight) follower.algorithm();
            // Past these errors Pedro spends power on turning / getting back on the path instead of driving
            if (Math.abs(foresight.headingError()) > Math.toRadians(11.25)) statTurnTime += dt;
            if (Math.abs(foresight.translationalError()) > 2.5) statOffPathTime += dt;
            if (foresight.isBraking()) statBrakeTime += dt;
        }
    }

    private void reportPath(double t, boolean timedOut) {
        String report = String.format(Locale.US,
                "%s %.1fs avg %.0f max %.0f in/s turning %.0f%% off path %.0f%% braking %.0f%%%s",
                pathState, t, statDistance / t, statMaxSpeed,
                100 * statTurnTime / t, 100 * statOffPathTime / t, 100 * statBrakeTime / t,
                timedOut ? " TIMED OUT" : "");
        Log.i("PathReport", report);
        pathReports.add(report);
    }

    protected void primeFlywheel() {
        mechanisms.spinUpFlywheel(SHOT_RPM);
    }

    // First volley spin up, timed from auto start so it keeps going through the MOVE path
    protected boolean firstSpinUpDone() {
        double t = opModeTimer.seconds();
        mechanisms.startShoot(FIRST_SHOT_RPM);
        boolean done = t >= SPIN_UP_TIME;
        if (done && pathState == PathState.SHOOT_PRELOAD1) {
            Log.i("ThreeCycleAutoBase", String.format(Locale.US, "%s feeding at %.1f rpm after %.2fs%s",
                    pathState, mechanisms.getFlywheelRPM(), t,
                    mechanisms.isFlywheelRamping() ? " (still ramping)" : ""));
        }
        return done;
    }

    // Returns true when the volley is done, the flywheel keeps spinning for the next volley
    protected boolean runShoot(double feedTime) {
        double t = pathTimer.seconds();
        // Flywheel is already at speed for volleys 2 and 3, so feed right away and keep the intake running
        if (feedStartTime < 0 && pathState != PathState.SHOOT_PRELOAD1) {
            mechanisms.spinUpFlywheel(SHOT_RPM);
            feedStartTime = t;
            Log.i("ThreeCycleAutoBase", String.format(Locale.US, "%s feeding at %.1f rpm",
                    pathState, mechanisms.getFlywheelRPM()));
        }
        if (feedStartTime < 0) {
            if (firstSpinUpDone()) {
                feedStartTime = t;
            }
            return false;
        }
        mechanisms.shoot();
        if (t >= feedStartTime + feedTime) {
            primeFlywheel();
            mechanisms.intakeKeepFlywheel();
            return true;
        }
        return false;
    }

    // ExtraLoad overrides this
    protected PathState afterThirdVolley() {
        return PathState.SHOOT_POS2_PARK_POS;
    }

    // ExtraLoad autos handle the EXTRA_ states here
    protected void handleExtraState() {
    }

    public void statePathUpdate() {
        // Running late, go park
        if (opModeTimer.seconds() > PARK_DEADLINE
                && pathState != PathState.SHOOT_POS2_PARK_POS
                && pathState != PathState.DONE) {
            Log.w("ThreeCycleAutoBase", "past park deadline, going to park");
            mechanisms.intakeKeepFlywheel();
            setPathState(PathState.SHOOT_POS2_PARK_POS);
        }

        switch (pathState) {
            case MOVE:
                firstSpinUpDone();
                if (runPath(() -> lineTo(getShootPose1(), getShootPose1_0()), SHOOT1_TO_SHOOT1_1_TIMEOUT)) {
                    setPathState(PathState.SHOOT_PRELOAD1);
                }
                break;

            case SHOOT_PRELOAD1:
                if (runShoot(FEED_TIME_1)) {
                    setPathState(PathState.SHOOT_POS1_SHOOT_POS_1_1);
                }
                break;

            case SHOOT_POS1_SHOOT_POS_1_1:
                if (runPath(() -> lineTo(getShootPose1(), getShootPose1_1()),
                        SHOOT1_TO_SHOOT1_1_TIMEOUT)) {
                    setPathState(PathState.SHOOT_POS1_1_INTAKE_POS);
                }
                break;

            case SHOOT_POS1_1_INTAKE_POS:
                if (runPath(this::buildIntakePath, SHOOT1_1_TO_INTAKE_TIMEOUT)) {
                    setPathState(PathState.INTAKE1);
                }
                break;

            case INTAKE1:
                mechanisms.intakeKeepFlywheel();
                if (pathTimer.seconds() >= INTAKE1_TIME) {
                    mechanisms.raiseFlower();
                    setPathState(PathState.INTAKE_POS_SHOOT_POS2);
                }
                break;

            case INTAKE_POS_SHOOT_POS2:
                if (pathTimer.seconds() > 0.5) {
                    mechanisms.stopIntake();
                }
                primeFlywheel();
                if (pathTimer.seconds() > 2.5) {
                    mechanisms.lowerFlower();
                }
                if (runPath(this::buildShoot2Path, INTAKE_TO_SHOOT2_TIMEOUT)) {
                    mechanisms.lowerFlower();
                    setPathState(PathState.SHOOT_PRELOAD2);
                }
                break;

            case SHOOT_PRELOAD2:
                if (runShoot(FEED_TIME_2)) {
                    setPathState(PathState.PRE_SHOOT_POS2_SHOOT_POS2);
                }
                break;

            case PRE_SHOOT_POS2_SHOOT_POS2:
                mechanisms.intakeKeepFlywheel();
                if (runPath(() -> lineTo(getPreShootPose2(), getShootPose2()),
                        PRE_SHOOT2_TO_SHOOT2_TIMEOUT)) {
                    // Start feeding the moment the robot gets to shoot pose 2
                    setPathState(PathState.SHOOT_PRELOAD3);
                }
                break;

            case INTAKE2:
                mechanisms.intakeKeepFlywheel();
                if (pathTimer.seconds() >= 0.5) {
                    setPathState(PathState.SHOOT_PRELOAD3);
                }
                break;

            case SHOOT_PRELOAD3:
                if (runShoot(FEED_TIME_3)) {
                    setPathState(afterThirdVolley());
                }
                break;

            case SHOOT_POS2_PARK_POS:
                if (pathTimer.seconds() > 0.1) {
                    mechanisms.raiseFlower();
                }
                if (runPath(this::buildParkPath, SHOOT2_TO_PARK_TIMEOUT)) {
                    setPathState(PathState.DONE);
                }
                break;

            case DONE:
                mechanisms.reset();
                if (!follower.isBusy()) {
                    follower.stop();
                }
                break;

            default:
                handleExtraState();
                break;
        }
    }

    public void setPathState(PathState newState) {
        Log.i("ThreeCycleAutoBase", "state " + pathState + " -> " + newState);
        pathState = newState;
        pathStarted = false;
        feedStartTime = -1;
        pathTimer.reset();
    }

    @Override
    public void init() {
        Log.i("ThreeCycleAutoBase", "init");
        // Bulk caching to shorten loop time
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        mechanisms = new AutoMechanisms(hardwareMap);
        follower.setPose(getShootPose1());
        mechanisms.raiseFlower();
        pathState = PathState.MOVE;
    }

    @Override
    public void start() {
        opModeTimer.reset();
        lastLogTime = 0;
        pathReports.clear();
        setPathState(PathState.MOVE);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();
        mechanisms.update();
        // Kept up to date every loop so teleop gets the pose even if stop() is skipped
        TeleOpData.autoEndPose = follower.pose();

        telemetry.addData("Path State", pathState.toString());
        telemetry.addData("Follower busy", follower.isBusy());
        telemetry.addData("x", follower.pose().x());
        telemetry.addData("y", follower.pose().y());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Path Time", pathTimer.seconds());
        telemetry.addData("Auto Time", opModeTimer.seconds());
        telemetry.addData("intake motor", mechanisms.intakeMotor.getVelocity());
        telemetry.addData("flywheel rpm", mechanisms.getFlywheelRPM());
        for (String report : pathReports) {
            telemetry.addLine(report);
        }
        telemetry.update();

        // Only log a few times a second
        double now = opModeTimer.seconds();
        if (now - lastLogTime >= 0.25) {
            lastLogTime = now;
            Log.i("loop", "state=" + pathState + " x=" + follower.pose().x()
                    + " y=" + follower.pose().y()
                    + " heading=" + follower.pose().heading()
                    + " pathTime=" + pathTimer.seconds()
                    + " busy=" + follower.isBusy());
        }
    }

    @Override
    public void stop() {
        // Teleop starts its field position from here
        TeleOpData.autoEndPose = follower.pose();
        mechanisms.reset();
        follower.stop();
    }
}
