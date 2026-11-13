package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Autonomous.Essentials.Constants;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import android.util.Log;

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
        EXTRA_PICKUP
    }

    // Shooting timings (seconds)
    protected static final double SPIN_UP_TIME = 0.8;
    protected static final double PRESPUN_SPIN_UP_TIME = 0.3;
    protected static final double FEED_TIME = 3.2;
    protected static final double PRESPIN_AFTER = 1.0;

    // Wait this long before trusting follower.isBusy() after starting a path
    protected static final double MIN_PATH_TIME = 0.1;

    // Max seconds per path before moving on
    protected static final double SHOOT1_TO_SHOOT1_1_TIMEOUT = 2.0;
    protected static final double SHOOT1_1_TO_INTAKE_TIMEOUT = 4.0;
    protected static final double INTAKE_TO_SHOOT2_TIMEOUT = 6.0;
    protected static final double PRE_SHOOT2_TO_SHOOT2_TIMEOUT = 1.5;
    protected static final double SHOOT2_TO_PARK_TIMEOUT = 5.0;

    // Skip to park after this many seconds
    protected static final double PARK_DEADLINE = 24.5;

    protected PathState pathState;

    // Path for this state already handed to the follower
    private boolean pathStarted = false;
    // Flywheel already spinning for the next volley
    private boolean flywheelPrimed = false;
    private double lastLogTime = 0;

    // Abstract pose getters to be implemented by Red/Blue subclasses
    protected abstract Pose getShootPose1();
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

    // ExtraLoad overrides this
    protected Path buildParkPath() {
        return curveTo(getShootPose2(), getParkControl(), getParkPose());
    }

    // Starts the path once, then returns true when it is done or timed out
    protected boolean runPath(Supplier<Path> pathSupplier, double timeout) {
        if (!pathStarted) {
            follower.follow(pathSupplier.get());
            pathStarted = true;
            Log.i(pathState.toString(), "path started");
            return false;
        }
        double t = pathTimer.seconds();
        boolean timedOut = t > timeout;
        if (timedOut) Log.w(pathState.toString(), "path timed out after " + t + "s");
        return (t > MIN_PATH_TIME && !follower.isBusy()) || timedOut;
    }

    protected void primeFlywheel() {
        mechanisms.spinUpFlywheel();
        flywheelPrimed = true;
    }

    // Returns true when the volley is done
    protected boolean runShoot() {
        double t = pathTimer.seconds();
        double spinUp = flywheelPrimed ? PRESPUN_SPIN_UP_TIME : SPIN_UP_TIME;
        if (t < spinUp) {
            mechanisms.startShoot();
        } else {
            mechanisms.shoot();
        }
        if (t >= spinUp + FEED_TIME) {
            mechanisms.intake();
            flywheelPrimed = false;
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
            mechanisms.intake();
            setPathState(PathState.SHOOT_POS2_PARK_POS);
        }

        switch (pathState) {
            case SHOOT_PRELOAD1:
                if (runShoot()) {
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
                if (runPath(() -> curveTo(getShootPose1_1(), getIntakeControl(), getIntakePose()),
                        SHOOT1_1_TO_INTAKE_TIMEOUT)) {
                    mechanisms.lowerFlower();
                    setPathState(PathState.INTAKE1);
                }
                break;

            case INTAKE1:
                mechanisms.intake();
                if (pathTimer.seconds() >= 0.25) {
                    mechanisms.raiseFlower();
                    setPathState(PathState.INTAKE_POS_SHOOT_POS2);
                }
                break;

            case INTAKE_POS_SHOOT_POS2:
                if (pathTimer.seconds() > PRESPIN_AFTER) {
                    primeFlywheel();
                }
                if (pathTimer.seconds() > 2.5) {
                    mechanisms.lowerFlower();
                }
                if (runPath(() -> curveTo(getIntakePose(), getShoot2Control(), getPreShootPose2()),
                        INTAKE_TO_SHOOT2_TIMEOUT)) {
                    mechanisms.lowerFlower();
                    setPathState(PathState.SHOOT_PRELOAD2);
                }
                break;

            case SHOOT_PRELOAD2:
                if (runShoot()) {
                    setPathState(PathState.PRE_SHOOT_POS2_SHOOT_POS2);
                }
                break;

            case PRE_SHOOT_POS2_SHOOT_POS2:
                if (runPath(() -> lineTo(getPreShootPose2(), getShootPose2()),
                        PRE_SHOOT2_TO_SHOOT2_TIMEOUT)) {
                    setPathState(PathState.INTAKE2);
                }
                break;

            case INTAKE2:
                mechanisms.intake();
                if (pathTimer.seconds() >= 0.5) {
                    setPathState(PathState.SHOOT_PRELOAD3);
                }
                break;

            case SHOOT_PRELOAD3:
                if (runShoot()) {
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
        pathState = PathState.SHOOT_PRELOAD1;
    }

    @Override
    public void start() {
        opModeTimer.reset();
        lastLogTime = 0;
        setPathState(PathState.SHOOT_PRELOAD1);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("Path State", pathState.toString());
        telemetry.addData("Follower busy", follower.isBusy());
        telemetry.addData("x", follower.pose().x());
        telemetry.addData("y", follower.pose().y());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Path Time", pathTimer.seconds());
        telemetry.addData("Auto Time", opModeTimer.seconds());
        telemetry.addData("intake motor", mechanisms.intakeMotor.getVelocity());
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
        mechanisms.reset();
        follower.stop();
    }
}
