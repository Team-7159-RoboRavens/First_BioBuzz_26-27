package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.pedropathing.utils.Angle;
import com.pedropathing.utils.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Autonomous.Essentials.Constants;
import org.firstinspires.ftc.teamcode.Limelight.HiveTracker;
import org.firstinspires.ftc.teamcode.Shooter.ShotCalculator;
import org.firstinspires.ftc.teamcode.Shooter.ShotCounter;
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
    // Saved HIVE state from the Limelight, null if the Limelight isn't there
    protected HiveTracker hive;
    private List<LynxModule> hubs = new ArrayList<>();

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
        // Pickup from the flower on the park wall (ExtraLoad, and after a return volley)
        EXTRA_APPROACH,
        EXTRA_INTAKE,
        EXTRA_PICKUP,
        // Back to the first shoot pose to shoot there again (HIVE hasn't tipped, and GardenCycle)
        RETURN_TO_SHOOT1,
        SHOOT_RETURN,
        // BackSide autos only
        WAIT_FOR_TIP,
        BACK_TO_PICKUP,

        MOVE
    }

    // Shooting timings (seconds)
    // Only the first volley waits for spin up, the flywheel then stays on for the whole auto
    // The first volley feeds once the flywheel has held its RPM for SPIN_UP_SETTLE_TIME, timed from auto start.
    // Never before MIN_SPIN_UP_TIME, always by SPIN_UP_TIME. Lower MIN until the first ball starts falling short
    protected static final double MIN_SPIN_UP_TIME = 0.5;
    protected static final double SPIN_UP_TIME = 4.7;
    // Flywheel counts as at speed within this many RPM of the target
    protected static final double RPM_TOLERANCE = 3;
    protected static final double SPIN_UP_SETTLE_TIME = 0.3;
    // Later volleys feed once the RPM is within RPM_TOLERANCE (volley 3 can use a different RPM than volley 2),
    // or after this many seconds
    protected static final double MAX_RPM_WAIT = 1.0;
    // How long each volley feeds
    protected static final double FEED_TIME_1 = 4.4;
    protected static final double FEED_TIME_2 = 3.8;
    protected static final double FEED_TIME_3 = 5.2;
    // One flower's worth of balls like volley 2
    protected static final double FEED_TIME_RETURN = FEED_TIME_2;

    // A volley ends early once this many balls are counted from flywheel RPM dips (Shooter/ShotCounter)
    protected static final int BALLS_PER_VOLLEY = 4;
    // When the feed time runs out first, strafe side to side to shake loose a stuck ball
    protected static final boolean JIGGLE_ENABLED = true;
    // Longest jiggle, it also stops as soon as another ball is counted
    protected static final double JIGGLE_TIME = 1.0;
    // First move is this far to the robot's right, then it goes twice this to the other side each move
    protected static final double JIGGLE_DISTANCE = 1.5;
    // Max seconds per jiggle move
    protected static final double JIGGLE_MOVE_TIMEOUT = 0.35;

    // ---------------- Limelight relocalize before each volley ----------------
    // At each shoot pose the robot reads its pose from the HIVE tags before feeding. With the tags in view the
    // follower pose is corrected, then the robot turns where it stands to the corrected aim heading and the RPM moves
    // with the distance change. It doesn't drive back onto the planned pose (shoot pose 2 is against the wall).
    // No tags in view = shoots as before after a short look. relocalizeBefore() turns it off per volley
    protected static final boolean RELOCALIZE_BEFORE_SHOT = true;
    // Frames averaged for one correction
    protected static final int RELOCALIZE_SAMPLES = 3;
    // Gives up after this many frames without a pose and none with one (tags not in view), or after this long
    protected static final int RELOCALIZE_GIVE_UP_FRAMES = 3;
    protected static final double RELOCALIZE_MAX_TIME = 0.4;
    // Frames taken while the robot moves faster than this are skipped, in/s and deg/s
    protected static final double RELOCALIZE_MAX_SPEED = 2;
    protected static final double RELOCALIZE_MAX_TURN_DEG = 10;
    // The frames have to agree this closely with their average or the correction is skipped, inches and degrees
    protected static final double RELOCALIZE_MAX_SPREAD = 1.5;
    protected static final double RELOCALIZE_MAX_HEADING_SPREAD_DEG = 3;
    // A correction bigger than this is more likely a bad read than drift, so it is skipped, inches and degrees
    protected static final double RELOCALIZE_MAX_JUMP = 8;
    protected static final double RELOCALIZE_MAX_HEADING_JUMP_DEG = 8;
    // Also take the heading from the Limelight (false keeps the Pinpoint heading)
    protected static final boolean RELOCALIZE_HEADING = true;
    // Most the RPM can move from the planned volley RPM
    protected static final double RELOCALIZE_MAX_RPM_CHANGE = 10;
    // Turn to the corrected aim heading until within this, or for at most this long
    protected static final double REAIM_TOLERANCE_DEG = 1;
    protected static final double REAIM_TIMEOUT = 0.6;

    // Flywheel RPM for the preload volley, the RPM for volleys 2 and 3 is set per alliance (getShot2RPM / getShot3RPM)
    protected static final double FIRST_SHOT_RPM = 192;

    // Wait this long before trusting follower.isBusy() after starting a path
    protected static final double MIN_PATH_TIME = 0.1;

    // Max seconds per path before moving on (only there so a stuck path can't hang the auto)
    protected static final double SHOOT1_TO_SHOOT1_1_TIMEOUT = 3.0;
    protected static final double SHOOT1_1_TO_INTAKE_TIMEOUT = 5.0;
    protected static final double INTAKE_TO_SHOOT2_TIMEOUT = 8.0;
    protected static final double PRE_SHOOT2_TO_SHOOT2_TIMEOUT = 2.0;
    protected static final double SHOOT2_TO_PARK_TIMEOUT = 6.0;
    protected static final double RETURN_TO_SHOOT1_TIMEOUT = 5.0;
    protected static final double BACK_TO_PICKUP_TIMEOUT = 3.0;
    protected static final double EXTRA_APPROACH_TIMEOUT = 5.0;
    protected static final double EXTRA_INTAKE_TIMEOUT = 2.0;

    // Skip to park after this many seconds
    protected static final double PARK_DEADLINE = 27.3;

    // Seconds to sit at the flower with the intake running, raise if balls are left behind
    protected static final double INTAKE1_TIME = 0.5;
    // Same as INTAKE2
    protected static final double EXTRA_PICKUP_TIME = 0.5;
    // How close to its end the intake path has to get before it counts as done (Pedro default is 0.025)
    protected static final double INTAKE_END_T = 0.01;

    // Face the direction of travel in the middle of the intake to shoot 2 path (false = old linear heading)
    protected static final boolean FACE_TRAVEL_TO_SHOOT2 = true;
    // Same for every other path at least FACE_TRAVEL_MIN_LENGTH inches long (not the flower intake path).
    // The robot drives with its front or its back leading, whichever needs less turning. false = old linear heading
    protected static final boolean FACE_TRAVEL = true;
    protected static final double FACE_TRAVEL_MIN_LENGTH = 30;
    // Turn to the travel direction by this much of the path, and start turning to the end heading here
    protected static final double TURN_IN_END = 0.25;
    protected static final double TURN_OUT_START = 0.7;

    protected PathState pathState;

    // Path for this state already handed to the follower
    private boolean pathStarted = false;
    // Volley state, reset on every state change: when feeding started
    private double feedStartTime = -1;
    // Auto time the flywheel got within RPM_TOLERANCE of its target, -1 while it isn't
    private double atSpeedSince = -1;
    // Made once the mechanisms exist, it reads the flywheel current
    private ShotCounter shotCounter;
    // Jiggle state for the current volley, jiggleStartTime < 0 until it starts
    private double jiggleStartTime = -1;
    private double jiggleMoveTime;
    private int jiggleShots;
    private int jiggleMove;
    private Pose jiggleCenter;
    // Limelight relocalize state for the current volley, reset on every state change
    private enum RelocalizeStage { LOOKING, REAIMING, DONE }
    private RelocalizeStage relocalizeStage = RelocalizeStage.LOOKING;
    private double relocalizeStartTime = -1;
    private int relocalizeCount;
    private int relocalizeEmptyFrames;
    private final double[] relocalizeDx = new double[RELOCALIZE_SAMPLES];
    private final double[] relocalizeDy = new double[RELOCALIZE_SAMPLES];
    private final double[] relocalizeDh = new double[RELOCALIZE_SAMPLES];
    private double reaimHeading;
    private double reaimStartTime;
    // Added to this volley's RPM after a correction
    private double rpmCorrection = 0;
    private double lastLogTime = 0;
    // Reused for the ShotRPM log line
    private final StringBuilder shotLogLine = new StringBuilder(96);
    // Planned pose the robot is at or last drove to, paths that can start from different places start from here
    protected Pose here;
    // Extra flower already picked up
    protected boolean extraDone = false;

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
    // Extra flower on the park wall: drive to the approach pose, then intake into the end pose
    protected abstract Pose getExtraApproachPose();
    protected abstract Pose getExtraIntakeEndPose();
    // Park from the extra flower curves through this so the robot can turn away from the wall
    protected abstract Pose getExtraParkControl();
    // RPM for volley 2 (at pre-shoot pose 2) and volley 3 (at shoot pose 2)
    protected abstract double getShot2RPM();
    protected abstract double getShot3RPM();
    protected abstract boolean isRed();

    // ---------------- Hooks the other autos change ----------------

    protected Pose getStartPose() { return getShootPose1(); }

    // Where the preload volley and the return volleys are shot from
    protected Pose getFirstShotPose() { return getShootPose1_0(); }

    protected double getFirstShotRPM() { return FIRST_SHOT_RPM; }

    protected PathState getFirstState() { return PathState.MOVE; }

    protected PathState afterFirstVolley() { return PathState.SHOOT_POS1_SHOOT_POS_1_1; }

    // Go back and shoot at the first shoot pose when its CELL is still up after the garden pickup
    protected boolean returnIfNotTipped() { return true; }

    // Safe autos don't need it
    protected boolean usesLimelight() { return true; }

    // After the garden pickup: back to the first shoot pose only if its CELL is still up (not moving or down)
    protected PathState afterGardenIntake() {
        if (!returnIfNotTipped() || hive == null) {
            return PathState.INTAKE_POS_SHOOT_POS2;
        }
        // Recheck right before deciding, the saved state is kept if the tags aren't in view
        hive.check();
        int state = hive.getState(getFirstShotPose().y());
        Log.i("ThreeCycleAutoBase", "first shoot CELL is " + HiveTracker.name(state) + " after the garden pickup");
        return state == HiveTracker.UP ? PathState.RETURN_TO_SHOOT1 : PathState.INTAKE_POS_SHOOT_POS2;
    }

    // ExtraLoad and BackSide right park go to the extra flower instead
    protected PathState afterThirdVolley() {
        return PathState.SHOOT_POS2_PARK_POS;
    }

    // After a return volley, pick up the extra flower if it hasn't been yet, otherwise park
    protected PathState afterReturnShot() {
        return extraDone ? PathState.SHOOT_POS2_PARK_POS : PathState.EXTRA_APPROACH;
    }

    // GardenCycle goes back to shoot instead
    protected PathState afterExtraPickup() {
        return PathState.SHOOT_POS2_PARK_POS;
    }

    // Volleys that wait for the CELL they shoot at to be up
    protected boolean waitForUpBefore(PathState shot) {
        return shot == PathState.SHOOT_RETURN;
    }

    // Volleys that check the Limelight first. Return false for a shoot pose where the tags are never in view
    protected boolean relocalizeBefore(PathState shot) {
        return true;
    }

    protected double getParkDeadline() { return PARK_DEADLINE; }

    // ---------------- Paths ----------------

    // Paths are built from the robot's actual position when their state starts
    // Heading comes from the planned pose so it never wraps the long way

    protected Pose fromActual(Pose nominal) {
        Pose p = follower.pose();
        return new Pose(p.x(), p.y(), nominal.heading());
    }

    protected Path lineTo(Pose nominalStart, Pose end) {
        Pose start = fromActual(nominalStart);
        return withHeading(line(start, end), start, end);
    }

    protected Path curveTo(Pose nominalStart, Pose control, Pose end) {
        Pose start = fromActual(nominalStart);
        return withHeading(curve(start, control, end), start, end);
    }

    // Long paths face the travel direction, short ones turn evenly from start to end
    private Path withHeading(Path path, Pose start, Pose end) {
        if (FACE_TRAVEL && path.curve.length() >= FACE_TRAVEL_MIN_LENGTH) {
            return faceTravel(path, start, end);
        }
        return path.linear(start, end);
    }

    // Has to get closer to the flower than a normal path before it counts as done.
    // Keeps the even turn so the intake comes at the flower straight
    protected Path buildIntakePath() {
        Pose start = fromActual(getShootPose1_1());
        Pose end = getIntakePose();
        return curve(start, getIntakeControl(), end).linear(start, end)
                .with(Constants.foresightConfig.parametricTConstraint.at(INTAKE_END_T));
    }

    protected Path buildShoot2Path() {
        Pose start = fromActual(getIntakePose());
        Pose end = getPreShootPose2();
        Path path = curve(start, getShoot2Control(), end);
        if (!FACE_TRAVEL_TO_SHOOT2) {
            return path.linear(start, end);
        }
        return faceTravel(path, start, end);
    }

    // Mecanum is fastest driving straight forward or back, so turn to the travel direction, drive, then turn to the
    // end heading. Front or back leads, whichever needs less turning at the two ends
    protected Path faceTravel(Path path, Pose start, Pose end) {
        double turnInHeading = path.curve.tangent(path.curve.parameter(TURN_IN_END)).theta();
        double turnOutHeading = path.curve.tangent(path.curve.parameter(TURN_OUT_START)).theta();
        double frontTurn = turnSize(start.heading(), turnInHeading) + turnSize(turnOutHeading, end.heading());
        double backTurn = turnSize(start.heading(), turnInHeading + Math.PI)
                + turnSize(turnOutHeading + Math.PI, end.heading());
        Interpolator middle = Interpolator.tangent;
        if (backTurn < frontTurn) {
            middle = BACK_TANGENT;
            turnInHeading += Math.PI;
            turnOutHeading += Math.PI;
        }
        return path.heading(Interpolator.piecewise()
                .until(TURN_IN_END, Interpolator.linear(start.heading(), turnInHeading))
                .until(TURN_OUT_START, middle)
                .until(1.0, Interpolator.linear(turnOutHeading, end.heading())));
    }

    // Back toward the travel direction. Pedro's tangent.reverse() isn't geometric, so inside a piecewise it would
    // read the tangent at the wrong spot on a curve
    private static final Interpolator BACK_TANGENT =
            Interpolator.geometric((curve, t) -> Angle.normalize(curve.tangent(t).theta() + Math.PI));

    // Radians of turning from one heading to the other the short way
    private static double turnSize(double from, double to) {
        return Math.abs(Math.IEEEremainder(to - from, 2 * Math.PI));
    }

    protected static boolean samePlace(Pose a, Pose b) {
        return Math.abs(a.x() - b.x()) < 0.01 && Math.abs(a.y() - b.y()) < 0.01;
    }

    // Park from wherever the robot is: away from the wall from the extra flower, a straight line from the
    // first shoot side, otherwise the shared park curve
    protected Path buildParkPath() {
        if (samePlace(here, getExtraIntakeEndPose()) || samePlace(here, getExtraApproachPose())) {
            return curveTo(here, getExtraParkControl(), getParkPose());
        }
        if (samePlace(here, getStartPose()) || samePlace(here, getFirstShotPose())
                || samePlace(here, getShootPose1())) {
            return lineTo(here, getParkPose());
        }
        return curveTo(here, getParkControl(), getParkPose());
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

    // Same as runPath, and saves the end pose as where the robot is
    protected boolean runPathTo(Supplier<Path> pathSupplier, Pose end, double timeout) {
        boolean done = runPath(pathSupplier, timeout);
        if (done) here = end;
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

    // ---------------- Shooting ----------------

    // Spins the flywheel to the RPM of the next volley, volley 3's as soon as volley 2 has started
    protected void primeFlywheel() {
        mechanisms.spinUpFlywheel(nextShotRPM());
    }

    protected double nextShotRPM() {
        switch (pathState) {
            case RETURN_TO_SHOOT1:
            case SHOOT_RETURN:
                return getFirstShotRPM();
            case EXTRA_APPROACH:
            case EXTRA_INTAKE:
            case EXTRA_PICKUP:
                return afterExtraPickup() == PathState.RETURN_TO_SHOOT1 ? getFirstShotRPM() : getShot3RPM();
            case SHOOT_PRELOAD2:
                // Called once volley 2 is done, so this is for volley 3
            case PRE_SHOOT_POS2_SHOOT_POS2:
            case INTAKE2:
            case SHOOT_PRELOAD3:
            case SHOOT_POS2_PARK_POS:
            case DONE:
            case BACK_TO_PICKUP:
                return getShot3RPM();
            default:
                return getShot2RPM();
        }
    }

    // RPM for the volley shot in this state, plus the change from a Limelight correction
    protected double volleyRPM(PathState shot) {
        return plannedVolleyRPM(shot) + rpmCorrection;
    }

    protected double plannedVolleyRPM(PathState shot) {
        switch (shot) {
            case SHOOT_PRELOAD1:
            case SHOOT_RETURN:
                return getFirstShotRPM();
            case SHOOT_PRELOAD2:
                return getShot2RPM();
            default:
                return getShot3RPM();
        }
    }

    // First volley spin up, timed from auto start so it keeps going through the MOVE path
    protected boolean firstSpinUpDone() {
        double rpm = getFirstShotRPM() + rpmCorrection;
        mechanisms.startShoot(rpm);
        double t = opModeTimer.seconds();
        boolean settled = updateAtSpeed(rpm, t) >= SPIN_UP_SETTLE_TIME;
        return (settled && t >= MIN_SPIN_UP_TIME) || t >= SPIN_UP_TIME;
    }

    // Seconds the flywheel has been within RPM_TOLERANCE of the target, -1 if it isn't now. Call every loop
    private double updateAtSpeed(double targetRPM, double t) {
        boolean atSpeed = !mechanisms.isFlywheelRamping()
                && Math.abs(mechanisms.getFlywheelRPM() - targetRPM) <= RPM_TOLERANCE;
        if (!atSpeed) {
            atSpeedSince = -1;
            return -1;
        }
        if (atSpeedSince < 0) {
            atSpeedSince = t;
        }
        return t - atSpeedSince;
    }

    // True if the CELL on the robot's side is up, or there is no Limelight to tell
    protected boolean hiveUpHere() {
        return hive == null || hive.isUp(follower.pose().y());
    }

    // Reads the robot pose from the Limelight while it stands at the shoot pose, corrects the follower with it and
    // turns to the corrected aim. True once the robot is ready to feed: corrected, gave up, or not used here
    private boolean relocalizeForShot(double t) {
        if (relocalizeStage == RelocalizeStage.DONE) {
            return true;
        }
        if (relocalizeStage == RelocalizeStage.REAIMING) {
            double error = turnSize(follower.pose().heading(), reaimHeading);
            if (error <= Math.toRadians(REAIM_TOLERANCE_DEG) || t - reaimStartTime >= REAIM_TIMEOUT) {
                Log.i("Relocalize", String.format(Locale.US, "%s re-aimed in %.2fs, %.1f deg off",
                        pathState, t - reaimStartTime, Math.toDegrees(error)));
                relocalizeStage = RelocalizeStage.DONE;
                return true;
            }
            return false;
        }
        if (hive == null || !RELOCALIZE_BEFORE_SHOT || !relocalizeBefore(pathState)) {
            relocalizeStage = RelocalizeStage.DONE;
            return true;
        }
        if (relocalizeStartTime < 0) {
            relocalizeStartTime = t;
            relocalizeCount = 0;
            relocalizeEmptyFrames = 0;
            hive.setTrackPose(true);
            return false;
        }
        if (hive.takePoseFrame()) {
            Pose seen = hive.getFramePose();
            Pose p = follower.pose();
            boolean still = follower.velocity().linSpeed() <= RELOCALIZE_MAX_SPEED
                    && Math.abs(follower.velocity().omega) <= Math.toRadians(RELOCALIZE_MAX_TURN_DEG);
            if (seen == null) {
                relocalizeEmptyFrames++;
            } else if (still && relocalizeCount < RELOCALIZE_SAMPLES) {
                // How far the Limelight says the follower pose is off
                relocalizeDx[relocalizeCount] = seen.x() - p.x();
                relocalizeDy[relocalizeCount] = seen.y() - p.y();
                relocalizeDh[relocalizeCount] = Math.IEEEremainder(seen.heading() - p.heading(), 2 * Math.PI);
                relocalizeCount++;
            }
        }
        if (relocalizeCount >= RELOCALIZE_SAMPLES) {
            applyRelocalize(t);
            return relocalizeStage == RelocalizeStage.DONE;
        }
        boolean noTags = relocalizeCount == 0 && relocalizeEmptyFrames >= RELOCALIZE_GIVE_UP_FRAMES;
        if (noTags || t - relocalizeStartTime >= RELOCALIZE_MAX_TIME) {
            finishRelocalize(String.format(Locale.US, "gave up after %.2fs, %d good frames, %d without tags",
                    t - relocalizeStartTime, relocalizeCount, relocalizeEmptyFrames));
            return true;
        }
        return false;
    }

    private void applyRelocalize(double t) {
        double dx = 0, dy = 0, sumCos = 0, sumSin = 0;
        for (int i = 0; i < relocalizeCount; i++) {
            dx += relocalizeDx[i];
            dy += relocalizeDy[i];
            sumCos += Math.cos(relocalizeDh[i]);
            sumSin += Math.sin(relocalizeDh[i]);
        }
        dx /= relocalizeCount;
        dy /= relocalizeCount;
        double dh = Math.atan2(sumSin, sumCos);
        for (int i = 0; i < relocalizeCount; i++) {
            boolean spread = Math.hypot(relocalizeDx[i] - dx, relocalizeDy[i] - dy) > RELOCALIZE_MAX_SPREAD
                    || turnSize(relocalizeDh[i], dh) > Math.toRadians(RELOCALIZE_MAX_HEADING_SPREAD_DEG);
            if (spread) {
                finishRelocalize("frames don't agree, not corrected");
                return;
            }
        }
        if (Math.hypot(dx, dy) > RELOCALIZE_MAX_JUMP || Math.abs(dh) > Math.toRadians(RELOCALIZE_MAX_HEADING_JUMP_DEG)) {
            finishRelocalize(String.format(Locale.US, "correction too big (%.1f in, %.1f deg), not corrected",
                    Math.hypot(dx, dy), Math.toDegrees(dh)));
            return;
        }
        if (!RELOCALIZE_HEADING) {
            dh = 0;
        }
        Pose p = follower.pose();
        Pose corrected = new Pose(p.x() + dx, p.y() + dy, Angle.normalize(p.heading() + dh));
        follower.setPose(corrected);

        // Keep the hand tuned heading and RPM of the planned pose, only add the change the aim math gives for
        // shooting from the corrected spot instead
        boolean red = isRed();
        Pose plannedAim = aimedPose(here.x(), here.y(), red);
        Pose correctedAim = aimedPose(corrected.x(), corrected.y(), red);
        double headingChange = Math.IEEEremainder(correctedAim.heading() - plannedAim.heading(), 2 * Math.PI);
        reaimHeading = Angle.normalize(here.heading() + headingChange);
        double rpmChange = aimedRPM(correctedAim, red) - aimedRPM(plannedAim, red);
        rpmCorrection = Math.max(-RELOCALIZE_MAX_RPM_CHANGE, Math.min(RELOCALIZE_MAX_RPM_CHANGE, rpmChange));

        // Hold where the robot is: holding the planned pose would drive it there, which can be into the wall
        follower.hold(new Pose(corrected.x(), corrected.y(), reaimHeading));
        reaimStartTime = t;
        finishRelocalize(String.format(Locale.US,
                "moved %.1f, %.1f in, %.1f deg, aim %+.1f deg, rpm %+.1f",
                dx, dy, Math.toDegrees(dh), Math.toDegrees(headingChange), rpmCorrection));
        relocalizeStage = RelocalizeStage.REAIMING;
    }

    private void finishRelocalize(String why) {
        Log.i("Relocalize", pathState + " " + why);
        hive.setTrackPose(false);
        relocalizeStage = RelocalizeStage.DONE;
    }

    // Returns true when the volley is done, the flywheel keeps spinning for the next volley
    protected boolean runShoot(double feedTime) {
        double t = pathTimer.seconds();
        if (feedStartTime < 0) {
            // Runs alongside the spin up, the RPM it changes is used right away
            boolean placed = relocalizeForShot(t);
            boolean ready;
            if (pathState == PathState.SHOOT_PRELOAD1) {
                ready = firstSpinUpDone();
            } else {
                // Flywheel is usually already at speed for the later volleys, this only waits if it isn't
                mechanisms.spinUpFlywheel(volleyRPM(pathState));
                ready = updateAtSpeed(volleyRPM(pathState), opModeTimer.seconds()) >= 0 || t >= MAX_RPM_WAIT;
            }
            // Don't feed into a CELL that is down or moving
            if (waitForUpBefore(pathState) && !hiveUpHere()) {
                ready = false;
            }
            if (!ready || !placed) {
                return false;
            }
            feedStartTime = t;
            shotCounter.reset();
            jiggleStartTime = -1;
            Log.i("ThreeCycleAutoBase", String.format(Locale.US, "%s feeding at %.1f rpm at %.2fs%s",
                    pathState, mechanisms.getFlywheelRPM(), opModeTimer.seconds(),
                    mechanisms.isFlywheelRamping() ? " (still ramping)" : ""));
        }
        mechanisms.shoot();
        double rpm = mechanisms.getFlywheelRPM();
        double target = volleyRPM(pathState);
        shotCounter.update(rpm, target, t, !mechanisms.isFlywheelRamping());
        int shots = shotCounter.getShots();
        // Logged every loop while feeding, so built without String.format
        StringBuilder line = shotLogLine;
        line.setLength(0);
        line.append(pathState).append(" t=");
        appendFixed(line, t - feedStartTime, 1000);
        line.append(" rpm=");
        appendFixed(line, rpm, 10);
        line.append(" target=");
        appendFixed(line, target, 10);
        line.append(" shots=").append(shots);
        if (jiggleStartTime >= 0) line.append(" jiggle");
        Log.i("ShotRPM", line.toString());

        if (shots >= BALLS_PER_VOLLEY) {
            return endVolley("all " + shots + " balls counted");
        }
        if (t < feedStartTime + feedTime) {
            return false;
        }
        if (!JIGGLE_ENABLED) {
            return endVolley("feed time up with " + shots + " balls counted");
        }
        if (jiggleStartTime < 0) {
            // Feed time is up and a ball may be stuck, shake it loose
            jiggleStartTime = t;
            jiggleShots = shots;
            jiggleMove = 0;
            jiggleCenter = follower.pose();
            startJiggleMove(t);
            Log.i("ThreeCycleAutoBase", pathState + " jiggling with " + shots + " balls counted");
            return false;
        }
        if (shots > jiggleShots) {
            return endVolley("ball counted while jiggling, " + shots + " total");
        }
        if (t >= jiggleStartTime + JIGGLE_TIME) {
            return endVolley("jiggle time up with " + shots + " balls counted");
        }
        if ((t - jiggleMoveTime > MIN_PATH_TIME && !follower.isBusy()) || t - jiggleMoveTime > JIGGLE_MOVE_TIMEOUT) {
            jiggleMove++;
            startJiggleMove(t);
        }
        return false;
    }

    // Appends v rounded to 1 / scale (scale 10 = one decimal), like %.1f
    private static void appendFixed(StringBuilder sb, double v, long scale) {
        long r = Math.round(Math.abs(v) * scale);
        if (v < 0 && r != 0) sb.append('-');
        sb.append(r / scale);
        sb.append('.');
        long frac = r % scale;
        for (long d = scale / 10; d > 0; d /= 10) {
            sb.append((char) ('0' + frac / d % 10));
        }
    }

    private boolean endVolley(String why) {
        Log.i("ThreeCycleAutoBase", String.format(Locale.US, "%s done after %.2fs: %s",
                pathState, pathTimer.seconds() - feedStartTime, why));
        primeFlywheel();
        mechanisms.intakeKeepFlywheel();
        return true;
    }

    // Moves 1.5 in to the robot's right, then 1.5 in to the left of the center, then right again, and so on
    private void startJiggleMove(double t) {
        double offset = jiggleMove % 2 == 0 ? JIGGLE_DISTANCE : -JIGGLE_DISTANCE;
        double heading = jiggleCenter.heading();
        // Robot right is the heading turned 90 deg clockwise
        Pose target = new Pose(
                jiggleCenter.x() + Math.sin(heading) * offset,
                jiggleCenter.y() - Math.cos(heading) * offset,
                heading);
        Pose p = follower.pose();
        follower.follow(line(new Pose(p.x(), p.y(), heading), target).constant(heading));
        jiggleMoveTime = t;
    }

    // ---------------- States ----------------

    // Other autos handle their own states here
    protected void handleExtraState() {
    }

    public void statePathUpdate() {
        // Running late, go park
        if (opModeTimer.seconds() > getParkDeadline()
                && pathState != PathState.SHOOT_POS2_PARK_POS
                && pathState != PathState.DONE) {
            Log.w("ThreeCycleAutoBase", "past park deadline, going to park");
            mechanisms.intakeKeepFlywheel();
            setPathState(PathState.SHOOT_POS2_PARK_POS);
        }

        switch (pathState) {
            case WAIT_FOR_TIP:
                // Spin up while waiting for our CELL on this side to tip up
                firstSpinUpDone();
                if (hiveUpHere()) {
                    Log.i("ThreeCycleAutoBase", String.format(Locale.US, "HIVE tipped at %.2fs", opModeTimer.seconds()));
                    setPathState(PathState.MOVE);
                }
                break;

            case MOVE:
                firstSpinUpDone();
                if (runPathTo(() -> lineTo(getStartPose(), getFirstShotPose()), getFirstShotPose(),
                        SHOOT1_TO_SHOOT1_1_TIMEOUT)) {
                    setPathState(PathState.SHOOT_PRELOAD1);
                }
                break;

            case SHOOT_PRELOAD1:
                if (runShoot(FEED_TIME_1)) {
                    setPathState(afterFirstVolley());
                }
                break;

            case SHOOT_POS1_SHOOT_POS_1_1:
                if (runPathTo(() -> lineTo(getFirstShotPose(), getShootPose1_1()), getShootPose1_1(),
                        SHOOT1_TO_SHOOT1_1_TIMEOUT)) {
                    setPathState(PathState.SHOOT_POS1_1_INTAKE_POS);
                }
                break;

            case SHOOT_POS1_1_INTAKE_POS:
                if (runPathTo(this::buildIntakePath, getIntakePose(), SHOOT1_1_TO_INTAKE_TIMEOUT)) {
                    setPathState(PathState.INTAKE1);
                }
                break;

            case INTAKE1:
                mechanisms.intakeKeepFlywheel();
                if (pathTimer.seconds() >= INTAKE1_TIME) {
                    mechanisms.raiseFlower();
                    setPathState(afterGardenIntake());
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
                if (runPathTo(this::buildShoot2Path, getPreShootPose2(), INTAKE_TO_SHOOT2_TIMEOUT)) {
                    mechanisms.lowerFlower();
                    setPathState(PathState.SHOOT_PRELOAD2);
                }
                break;

            case BACK_TO_PICKUP:
                // From the BackSide first shot to the flower next to it, then PRE_SHOOT_POS2_SHOOT_POS2 picks it up
                mechanisms.intakeKeepFlywheel();
                primeFlywheel();
                if (runPathTo(() -> lineTo(here, getPreShootPose2()), getPreShootPose2(), BACK_TO_PICKUP_TIMEOUT)) {
                    mechanisms.lowerFlower();
                    setPathState(PathState.PRE_SHOOT_POS2_SHOOT_POS2);
                }
                break;

            case SHOOT_PRELOAD2:
                if (runShoot(FEED_TIME_2)) {
                    setPathState(PathState.PRE_SHOOT_POS2_SHOOT_POS2);
                }
                break;

            case PRE_SHOOT_POS2_SHOOT_POS2:
                mechanisms.intakeKeepFlywheel();
                if (runPathTo(() -> lineTo(getPreShootPose2(), getShootPose2()), getShootPose2(),
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

            case RETURN_TO_SHOOT1:
                if (pathTimer.seconds() > 0.5) {
                    mechanisms.stopIntake();
                }
                primeFlywheel();
                if (runPathTo(() -> lineTo(here, getFirstShotPose()), getFirstShotPose(),
                        RETURN_TO_SHOOT1_TIMEOUT)) {
                    setPathState(PathState.SHOOT_RETURN);
                }
                break;

            case SHOOT_RETURN:
                if (runShoot(FEED_TIME_RETURN)) {
                    setPathState(afterReturnShot());
                }
                break;

            case EXTRA_APPROACH:
                runExtraIntake();
                if (runPathTo(() -> lineTo(here, getExtraApproachPose()), getExtraApproachPose(),
                        EXTRA_APPROACH_TIMEOUT)) {
                    mechanisms.lowerFlower();
                    setPathState(PathState.EXTRA_INTAKE);
                }
                break;

            case EXTRA_INTAKE:
                runExtraIntake();
                if (runPathTo(() -> lineTo(getExtraApproachPose(), getExtraIntakeEndPose()),
                        getExtraIntakeEndPose(), EXTRA_INTAKE_TIMEOUT)) {
                    setPathState(PathState.EXTRA_PICKUP);
                }
                break;

            case EXTRA_PICKUP:
                runExtraIntake();
                if (pathTimer.seconds() >= EXTRA_PICKUP_TIME) {
                    extraDone = true;
                    setPathState(afterExtraPickup());
                }
                break;

            case SHOOT_POS2_PARK_POS:
                if (pathTimer.seconds() > 0.1) {
                    mechanisms.raiseFlower();
                }
                if (runPathTo(this::buildParkPath, getParkPose(), SHOOT2_TO_PARK_TIMEOUT)) {
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

    // Keeps the flywheel up only if these balls get shot
    private void runExtraIntake() {
        if (afterExtraPickup() == PathState.SHOOT_POS2_PARK_POS) {
            mechanisms.intake();
        } else {
            mechanisms.intakeKeepFlywheel();
            primeFlywheel();
        }
    }

    public void setPathState(PathState newState) {
        Log.i("ThreeCycleAutoBase", "state " + pathState + " -> " + newState);
        pathState = newState;
        pathStarted = false;
        feedStartTime = -1;
        relocalizeStage = RelocalizeStage.LOOKING;
        relocalizeStartTime = -1;
        rpmCorrection = 0;
        if (hive != null) {
            hive.setTrackPose(false);
        }
        pathTimer.reset();
    }

    // ---------------- OpMode ----------------

    @Override
    public void init() {
        Log.i("ThreeCycleAutoBase", "init");
        // Bulk caching to shorten loop time. MANUAL does one hub read per loop, AUTO does another one every time
        // the same motor is read twice in a loop (the flywheel RPM is read several times)
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        mechanisms = new AutoMechanisms(hardwareMap);
        shotCounter = new ShotCounter(mechanisms.getFlywheel());
        follower.setPose(getStartPose());
        here = getStartPose();
        mechanisms.raiseFlower();
        hive = null;
        if (usesLimelight()) {
            try {
                hive = new HiveTracker(hardwareMap, isRed());
            } catch (RuntimeException e) {
                // No Limelight in the config: every CELL counts as up and the three cycle never returns
                Log.e("ThreeCycleAutoBase", "no Limelight, HIVE checks are off", e);
            }
        }
        pathState = getFirstState();
    }

    @Override
    public void init_loop() {
        clearBulkCache();
        // Keeps the saved HIVE state current before the start
        if (hive != null) {
            hive.update();
            telemetry.addData("HIVE CELL at start", HiveTracker.name(hive.getState(getStartPose().y())));
        } else if (usesLimelight()) {
            telemetry.addLine("No Limelight, HIVE checks are off");
        }
        telemetry.update();
    }

    // MANUAL bulk caching: new motor readings once per loop
    private void clearBulkCache() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }

    @Override
    public void start() {
        opModeTimer.reset();
        lastLogTime = 0;
        pathReports.clear();
        here = getStartPose();
        extraDone = false;
        setPathState(getFirstState());
    }

    @Override
    public void loop() {
        clearBulkCache();
        follower.update();
        if (hive != null) {
            hive.update();
        }
        statePathUpdate();
        mechanisms.update();
        // Kept up to date every loop so teleop gets the pose even if stop() is skipped
        Pose pose = follower.pose();
        TeleOpData.autoEndPose = pose;

        telemetry.addData("Path State", pathState.toString());
        telemetry.addData("Follower busy", follower.isBusy());
        telemetry.addData("x", pose.x());
        telemetry.addData("y", pose.y());
        telemetry.addData("Heading (deg)", Math.toDegrees(pose.heading()));
        telemetry.addData("Path Time", pathTimer.seconds());
        telemetry.addData("Auto Time", opModeTimer.seconds());
        telemetry.addData("intake motor", mechanisms.intakeMotor.getVelocity());
        telemetry.addData("flywheel rpm", mechanisms.getFlywheelRPM());
        if (hive != null) {
            telemetry.addData("HIVE CELL here", HiveTracker.name(hive.getState(pose.y())));
            telemetry.addData("HIVE CELL first shot", HiveTracker.name(hive.getState(getFirstShotPose().y())));
        }
        for (String report : pathReports) {
            telemetry.addLine(report);
        }
        telemetry.update();

        // Only log a few times a second
        double now = opModeTimer.seconds();
        if (now - lastLogTime >= 0.25) {
            lastLogTime = now;
            Log.i("loop", "state=" + pathState + " x=" + pose.x()
                    + " y=" + pose.y()
                    + " heading=" + pose.heading()
                    + " pathTime=" + pathTimer.seconds()
                    + " busy=" + follower.isBusy()
                    + (hive != null ? " hiveHere=" + HiveTracker.name(hive.getState(pose.y())) : ""));
        }
    }

    @Override
    public void stop() {
        // Teleop starts its field position from here
        TeleOpData.autoEndPose = follower.pose();
        mechanisms.reset();
        follower.stop();
        if (hive != null) {
            hive.stop();
        }
    }

    // Pose at (x, y) turned so the shooter faces our CELL on that side, from the teleop aim math
    protected static Pose aimedPose(double x, double y, boolean red) {
        double heading = 0;
        // The shooter is off the robot's center, so the heading takes a few passes to settle
        for (int i = 0; i < 3; i++) {
            heading = ShotCalculator.solve(x, y, heading, 0, 0, 0, 0, 0, red).heading;
        }
        return new Pose(x, y, Math.IEEEremainder(heading, 2 * Math.PI));
    }

    // Standing still RPM for a shot from this pose, from the teleop RPM math
    protected static double aimedRPM(Pose pose, boolean red) {
        return ShotCalculator.solve(pose.x(), pose.y(), pose.heading(), 0, 0, 0, 0, 0, red).rpm;
    }
}
