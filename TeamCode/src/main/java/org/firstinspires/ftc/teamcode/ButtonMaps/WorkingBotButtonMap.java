package org.firstinspires.ftc.teamcode.ButtonMaps;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Limelight.Limelight;
import org.firstinspires.ftc.teamcode.Limelight.LimelightBlue;
import org.firstinspires.ftc.teamcode.Localization.KalmanPositionFilter;
import org.firstinspires.ftc.teamcode.MotorPowers;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.BallCounter;
import org.firstinspires.ftc.teamcode.IntakeBallDetector;
import org.firstinspires.ftc.teamcode.Shooter.ShotCalculator;
import org.firstinspires.ftc.teamcode.Shooter.ShotCounter;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOpData;

import java.util.Locale;

// Gamepad 1: drive, hold RT to aim at our HIVE, hold LT to strafe along the shooting line while aiming,
//            A = turn the intake to face the nearest flower, hold left bumper = wiggle side to side to free a stuck
//            flower, hold right bumper = shoot, back = relocalize from the Limelight
// Gamepad 2: triggers intake, bumpers transfer, x flower, back = make the current facing forward,
//            dpad up fixed RPM / down automatic RPM, ball count: dpad right +1, dpad left -1, B = 4, A = 0
public class WorkingBotButtonMap extends PinpointBM {
    // ---------------- Tuning: turning to aim ----------------
    // Turn power per radian of aim error. Higher = snappier aim, too high wobbles around the target
    public static double ROTATE_KP = 1.2;
    // Turn power taken off per rad/s of turning, slows the turn before it gets there so it doesn't swing past.
    // Raise it if the aim still overshoots, then raise ROTATE_KP for a faster aim. 0 turns it off
    public static double ROTATE_KD = 0.1;
    public static double ROTATE_MAX_POWER = 0.7;
    // Smallest turn power used outside the tolerance, so it doesn't stall just short of the target
    public static double ROTATE_MIN_POWER = 0.06;
    public static double ROTATE_TOLERANCE_DEG = 1;

    // ---------------- Tuning: shooting (g1 right bumper) ----------------
    // Transfer and intake power while g1 right bumper is held (same direction as g2 right bumper / right trigger)
    public static double SHOOT_TRANSFER_POWER = 0.7;
    public static double SHOOT_INTAKE_POWER = 0.6;

    // ---------------- Tuning: strafing (LT) ----------------
    // The strafe line runs along x, this many inches from the field center (along y) on the robot's side, measured
    // to the robot's center. Bigger = strafes further from the HIVE. Auto shot 1 is at about 52
    public static double STRAFE_LINE_DISTANCE = 55;
    // How far the robot strafes to the left and to the right of our HIVE, as seen from the shooter looking at it, inches
    public static double STRAFE_LEFT = 12;
    public static double STRAFE_RIGHT = 12;
    // Power along the line while strafing. Faster makes the moving shot math matter more
    public static double STRAFE_POWER = 0.45;
    // Getting onto the line: power per inch away, the most power used, and how close counts as on the line
    public static double APPROACH_KP = 0.08;
    public static double APPROACH_MAX_POWER = 1.0;
    public static double APPROACH_TOLERANCE = 2;
    // Power per inch off the line while strafing, keeps it on the line
    public static double LINE_KP = 0.05;
    // Turns around this many inches before the end of the line (the robot slides a little while reversing)
    public static double END_TOLERANCE = 1.5;

    // ---------------- Tuning: wiggle (left bumper) ----------------
    // Goes this far right first, then twice this far left, then twice this far right, and so on (inches)
    public static double WIGGLE_DISTANCE = 1.5;
    // Power per inch from the wiggle target, the most and least power used. Raise the max for a harder shake
    public static double WIGGLE_KP = 0.25;
    public static double WIGGLE_MAX_POWER = 0.6;
    public static double WIGGLE_MIN_POWER = 0.15;
    // Turns around this close to the target, inches
    public static double WIGGLE_TOLERANCE = 0.3;
    // Turns around after this many seconds even if it didn't get there (stuck on the flower)
    public static double WIGGLE_LEG_TIMEOUT = 0.5;

    // ---------------- Tuning: shooter ----------------
    // RPM used with dpad up (the old fixed value), and when the automatic RPM can't be worked out
    public static double FIXED_RPM = 192;

    // ---------------- Tuning: Limelight ----------------
    // Most Limelight position solves per second, more just adds loop time
    public static double LIMELIGHT_SOLVES_PER_SECOND = 15;
    // Limelight poses averaged together for a g1 back relocalize
    public static int RELOCALIZE_SAMPLES = 5;
    // Also take the heading from the Limelight on relocalize (false keeps the Pinpoint heading)
    public static boolean RELOCALIZE_HEADING = true;

    // ---------------- Tuning: speed estimate ----------------
    // Smoothing time for the speed / acceleration, seconds. Same at any loop time (0 = no smoothing, higher = smoother but slower)
    // The old per loop values 0.5 / 0.2 were about these at 20 ms loops
    public static double VELOCITY_SMOOTHING_TIME = 0.03;
    public static double ACCELERATION_SMOOTHING_TIME = 0.09;

    // ---------------- Tuning: ball count ----------------
    // Count balls leaving from flywheel dips while the transfer feeds (ShotCounter has the thresholds)
    public static boolean COUNT_SHOTS = true;
    // Intake power above this counts as g2 intaking
    public static double INTAKE_HELD_POWER = 0.05;

    private static final double TRIGGER_PRESSED = 0.3;
    private static final double MM_PER_INCH = 25.4;

    private final Robot robot;
    private final Telemetry telemetry;
    private final boolean red;
    private final Limelight limelight;
    // getPosition() uses every HIVE tag, so the Blue class works for both alliances
    private final LimelightBlue limelightPosition;
    private final KalmanPositionFilter positionFilter;
    private long previousUpdateNanos = System.nanoTime();
    private long lastSolveNanos = 0;
    // Loop time for telemetry: this loop, a smoothed average and the slowest in the last second, seconds
    private double loopSeconds = 0, averageLoopSeconds = 0, maxLoopSeconds = 0, shownMaxLoopSeconds = 0;
    private long maxLoopWindowStartNanos = System.nanoTime();
    private boolean firstLoop = true;

    // Last finite Pinpoint reading (mm, mm, radians, mm/s)
    private double pinpointX = 0;
    private double pinpointY = 0;
    private double pinpointHeading = 0;
    private double pinpointVelX = 0;
    private double pinpointVelY = 0;
    private double pinpointTurnRate = 0;
    private boolean havePinpointReading = false;
    private double startPinpointHeading = 0;

    // Driving forward is away from our park wall, unless g2 back set a custom one (stored as a raw Pinpoint heading
    // so Limelight corrections don't turn it)
    private boolean customForward = false;
    private double customForwardPinpointHeading = 0;

    // g1 back relocalize: Limelight minus estimate offsets collected so far
    private boolean relocalizing = false;
    private int relocalizeCount = 0;
    private double relocalizeSumX, relocalizeSumY, relocalizeSumCos, relocalizeSumSin;

    // Smoothed field velocity (in/s) and acceleration (in/s^2)
    private double velX, velY, accelX, accelY;
    // Smoothed turn rate, rad/s counterclockwise
    private double turnRate;

    // A: field heading the intake is turning to
    private boolean flowerTurning = false;
    private double flowerTargetHeading = 0;

    // Left bumper wiggle: where it started, the robot's right at the start, and which side it is heading to
    private boolean wiggling = false;
    private double wiggleStartX, wiggleStartY, wiggleRightX, wiggleRightY;
    private int wiggleSide = 1;
    private long wiggleLegStartNanos = 0;

    private enum StrafeStage { APPROACH, STRAFE }
    private StrafeStage strafeStage = StrafeStage.APPROACH;
    // +1 strafing toward +x, -1 toward -x
    private int strafeDirection = 1;

    // Shooter modes, kept until the other button is pressed
    private boolean automaticRPM = true;

    // Balls in the robot, see BallCounter
    private final BallCounter ballCounter;
    private final IntakeBallDetector intakeBallDetector = new IntakeBallDetector();
    private final ShotCounter shotCounter = new ShotCounter();
    private int lastShots = 0;

    // Results of this loop's getPowers(), used by getState()
    private ShotCalculator.Shot shot = null;

    // Reused every loop so getPowers() / getState() don't make new objects
    private final MotorPowers drivePowers = new MotorPowers();
    private final double[] state = new double[4];
    // Field (x, y) power set by the wiggle / strafe
    private double fieldPowerX, fieldPowerY;

    public WorkingBotButtonMap(OpMode op, Robot r, boolean red) {
        super(op, r);
        robot = r;
        telemetry = op.telemetry;
        this.red = red;
        Limelight ll = null;
        try {
            ll = new Limelight(r);
            ll.useAprilTagPipeline();
        } catch (Exception e) {
            // No Limelight plugged in or configured, teleop still drives on the Pinpoint alone
            telemetry.addLine("Limelight not found, running without it");
        }
        limelight = ll;
        ballCounter = new BallCounter(g1, g2);
        limelightPosition = new LimelightBlue(limelight, telemetry);
        // Starts from where the auto ended. Without an auto the first Limelight pose sets the field position
        Pose autoEndPose = TeleOpData.autoEndPose;
        positionFilter = autoEndPose != null ? new KalmanPositionFilter(autoEndPose) : new KalmanPositionFilter();
        telemetry.addData("Start pose", autoEndPose != null
                ? String.format(Locale.US, "from auto (%.1f, %.1f, %.0f deg)",
                        autoEndPose.x(), autoEndPose.y(), Math.toDegrees(autoEndPose.heading()))
                : "no auto ran, waiting for the Limelight");
        // Used once, a later teleop run starts somewhere else
        TeleOpData.autoEndPose = null;
    }

    @Override
    public MotorPowers getPowers() {
        robot.pinpointUpdate();
        readPinpoint();
        KalmanPositionFilter.FieldPosition position = updatePosition();
        boolean fieldPoseKnown = positionFilter.hasFieldPose();

        if (g2.backWasPressed()) {
            customForward = true;
            customForwardPinpointHeading = pinpointHeading;
        }
        if (g1.backWasPressed() && limelight != null) {
            relocalizing = true;
            relocalizeCount = 0;
            relocalizeSumX = relocalizeSumY = relocalizeSumCos = relocalizeSumSin = 0;
        }

        double forward = getDpadForward(g1.dpad_up, g1.dpad_down);
        double right = getDpadRight(g1.dpad_left, g1.dpad_right);
        getMovement(g1.left_stick_x, g1.left_stick_y, stickMovement);
        forward += stickMovement[0];
        right += stickMovement[1];
        // Only the right stick turns, the bumpers don't
        double driverRotate = getRotation(g1.right_stick_x);
        boolean driverTranslating = forward != 0 || right != 0;
        boolean driverRotating = driverRotate != 0;

        boolean strafeHeld = g1.left_trigger > TRIGGER_PRESSED;
        boolean aimHeld = g1.right_trigger > TRIGGER_PRESSED;
        boolean aiming = (strafeHeld || aimHeld) && fieldPoseKnown;
        if (!strafeHeld || driverTranslating) {
            // Getting back onto the line first once the trigger is held again or the driver lets go
            strafeStage = StrafeStage.APPROACH;
        }

        shot = fieldPoseKnown ? ShotCalculator.solve(position.xInches, position.yInches, position.headingRadians,
                velX, velY, accelX, accelY, turnRate, red) : null;
        boolean shotValid = shot != null && shot.valid;
        double headingError = shotValid
                ? Math.IEEEremainder(shot.heading - position.headingRadians, 2 * Math.PI) : 0;

        if (g1.aWasPressed() && fieldPoseKnown) {
            flowerTurning = true;
            flowerTargetHeading = getNearestFlowerHeading(position.xInches - 72, position.yInches - 72);
        }
        if (driverRotating) {
            flowerTurning = false;
        }

        // Rotation: the driver always wins, then the A turn, otherwise aim while a trigger is held
        double rotate = driverRotate;
        if (flowerTurning) {
            double flowerError = Math.IEEEremainder(flowerTargetHeading - position.headingRadians, 2 * Math.PI);
            rotate = getAimRotationPower(flowerError);
            if (rotate == 0) {
                flowerTurning = false;
            }
        } else if (!driverRotating && aiming && shotValid) {
            rotate = getAimRotationPower(headingError);
        }

        telemetry.addData("Field pose", "known(%b) x(%.1f) y(%.1f) heading(%.1f deg)", fieldPoseKnown,
                position.xInches, position.yInches, Math.toDegrees(position.headingRadians));
        if (shotValid) {
            telemetry.addData("Shot", "dist %.1f in, rpm %.0f, aim err %.1f deg",
                    shot.distance, shot.rpm, Math.toDegrees(headingError));
        } else {
            telemetry.addData("Shot", "none");
        }
        telemetry.addData("Modes", "rpm %s%s", getRPMModeName(),
                relocalizing ? ", RELOCALIZING" : "");
        telemetry.addData("Loop", "%.1f ms, avg %.1f ms, max %.1f ms",
                loopSeconds * 1000, averageLoopSeconds * 1000, shownMaxLoopSeconds * 1000);

        // Translation: the driver always wins, then the wiggle, otherwise strafe while LT is held
        boolean wiggleHeld = g1.left_bumper && !driverTranslating;
        if (!wiggleHeld) {
            wiggling = false;
        }
        boolean useFieldPower = false;
        if (wiggleHeld) {
            setWiggleFieldPower(position);
            useFieldPower = true;
        } else if (strafeHeld && fieldPoseKnown && !driverTranslating) {
            setStrafeFieldPower(position);
            useFieldPower = true;
        }
        if (useFieldPower) {
            double cos = Math.cos(position.headingRadians);
            double sin = Math.sin(position.headingRadians);
            double robotForward = fieldPowerX * cos + fieldPowerY * sin;
            double robotRight = fieldPowerX * sin - fieldPowerY * cos;
            return Mechanumdrive.setMotorPowers(robotForward, robotRight, rotate, drivePowers);
        }
        return Mechanumdrive.fieldOrientedDrive(forward, right, rotate,
                Math.PI / 2 + getDriveHeading(position, fieldPoseKnown), drivePowers);
    }

    // Robot heading measured from the driving forward direction, radians
    private double getDriveHeading(KalmanPositionFilter.FieldPosition position, boolean fieldPoseKnown) {
        if (customForward) {
            return pinpointHeading - customForwardPinpointHeading;
        }
        if (fieldPoseKnown) {
            // Blue parks on the x = 144 wall so forward is -x, red parks on the x = 0 wall so forward is +x
            double forwardHeading = red ? 0 : Math.PI;
            return position.headingRadians - forwardHeading;
        }
        // No field position yet: forward is the way the robot faced when teleop started
        return pinpointHeading - startPinpointHeading;
    }

    // The lines y = x and y = -x split the field into 4 quadrants with one flower each, at the wall on the +x, +y, -x
    // or -y side. Returns the Pedro heading (intake end) that faces that wall
    public static double getNearestFlowerHeading(double x, double y) {
        if (Math.abs(x) > Math.abs(y)) {
            return x > 0 ? 0 : Math.PI;
        }
        return y > 0 ? Math.PI / 2 : -Math.PI / 2;
    }

    // Sets the field (x, y) power for the wiggle: right WIGGLE_DISTANCE from where it started, then back and forth
    // between WIGGLE_DISTANCE left and right of that start. Works without a field position, it only needs to move
    private void setWiggleFieldPower(KalmanPositionFilter.FieldPosition position) {
        long now = System.nanoTime();
        if (!wiggling) {
            wiggling = true;
            wiggleStartX = position.xInches;
            wiggleStartY = position.yInches;
            // Robot right is the heading turned 90 deg clockwise
            wiggleRightX = Math.sin(position.headingRadians);
            wiggleRightY = -Math.cos(position.headingRadians);
            wiggleSide = 1;
            wiggleLegStartNanos = now;
        }
        double along = (position.xInches - wiggleStartX) * wiggleRightX + (position.yInches - wiggleStartY) * wiggleRightY;
        double error = wiggleSide * WIGGLE_DISTANCE - along;
        boolean legTimedOut = (now - wiggleLegStartNanos) / 1_000_000_000.0 > WIGGLE_LEG_TIMEOUT;
        if (Math.abs(error) < WIGGLE_TOLERANCE || legTimedOut) {
            wiggleSide = -wiggleSide;
            wiggleLegStartNanos = now;
            error = wiggleSide * WIGGLE_DISTANCE - along;
        }
        double power = Math.max(WIGGLE_MIN_POWER, Math.min(WIGGLE_MAX_POWER, Math.abs(error) * WIGGLE_KP));
        power = Math.copySign(power, error);
        fieldPowerX = wiggleRightX * power;
        fieldPowerY = wiggleRightY * power;
    }

    private double getAimRotationPower(double error) {
        if (Math.abs(error) < Math.toRadians(ROTATE_TOLERANCE_DEG)) {
            return 0;
        }
        // Counterclockwise power: toward the target, minus some for how fast it already turns
        double ccw = error * ROTATE_KP - turnRate * ROTATE_KD;
        if (Math.signum(ccw) == Math.signum(error)) {
            // Still pushing toward the target, keep at least the min power so it doesn't stall
            ccw = Math.signum(error) * Math.max(ROTATE_MIN_POWER, Math.abs(ccw));
        }
        // Pushing the other way means it comes in too fast, that brakes it
        ccw = Math.max(-ROTATE_MAX_POWER, Math.min(ROTATE_MAX_POWER, ccw));
        // Headings grow counterclockwise and rotation power is clockwise
        return -ccw;
    }

    // Sets the field (x, y) power: get onto the strafe line as fast as possible, then go back and forth along it
    private void setStrafeFieldPower(KalmanPositionFilter.FieldPosition position) {
        double hiveX = 72 + (red ? -ShotCalculator.HIVE_X_FROM_CENTER : ShotCalculator.HIVE_X_FROM_CENTER);
        double side = position.yInches >= 72 ? 1 : -1;
        double lineY = 72 + side * STRAFE_LINE_DISTANCE;
        // Looking at the HIVE from the +y side the shooter's right is -x, from the -y side it is +x
        double rightX = hiveX - side * STRAFE_RIGHT;
        double leftX = hiveX + side * STRAFE_LEFT;
        double minX = Math.min(leftX, rightX);
        double maxX = Math.max(leftX, rightX);

        if (strafeStage == StrafeStage.APPROACH) {
            double targetX = Math.max(minX, Math.min(maxX, position.xInches));
            double errorX = targetX - position.xInches;
            double errorY = lineY - position.yInches;
            double distance = Math.hypot(errorX, errorY);
            if (distance > APPROACH_TOLERANCE) {
                double power = Math.min(APPROACH_MAX_POWER, distance * APPROACH_KP);
                fieldPowerX = errorX / distance * power;
                fieldPowerY = errorY / distance * power;
                return;
            }
            strafeStage = StrafeStage.STRAFE;
            // Head for the far end first so the first sweep is a full one
            strafeDirection = (maxX - position.xInches) > (position.xInches - minX) ? 1 : -1;
        }
        if (strafeDirection > 0 && position.xInches >= maxX - END_TOLERANCE) {
            strafeDirection = -1;
        } else if (strafeDirection < 0 && position.xInches <= minX + END_TOLERANCE) {
            strafeDirection = 1;
        }
        fieldPowerX = strafeDirection * STRAFE_POWER;
        fieldPowerY = Math.max(-0.5, Math.min(0.5, (lineY - position.yInches) * LINE_KP));
    }

    // Keeps the last good reading, a NaN from the Pinpoint would throw in the filter and stop the op mode
    private void readPinpoint() {
        double x = robot.getPinpointX();
        double y = robot.getPinpointY();
        double heading = robot.pinpointAngle();
        if (Double.isFinite(x) && Double.isFinite(y) && Double.isFinite(heading)) {
            pinpointX = x;
            pinpointY = y;
            pinpointHeading = heading;
            if (!havePinpointReading) {
                havePinpointReading = true;
                startPinpointHeading = heading;
            }
        }
        double vx = robot.getPinpointVelX();
        double vy = robot.getPinpointVelY();
        if (Double.isFinite(vx) && Double.isFinite(vy)) {
            pinpointVelX = vx;
            pinpointVelY = vy;
        }
        double w = robot.getPinpointHeadingVelocity();
        if (Double.isFinite(w)) {
            pinpointTurnRate = w;
        }
    }

    // Runs the filter on this loop's Pinpoint reading plus a Limelight pose when there is a new one
    private KalmanPositionFilter.FieldPosition updatePosition() {
        long now = System.nanoTime();
        double deltaTimeSeconds = (now - previousUpdateNanos) / 1_000_000_000.0;
        previousUpdateNanos = now;
        updateLoopTime(deltaTimeSeconds, now);

        Pose limelightPose = null;
        double latency = 0;
        if (limelight != null) {
            limelight.update();
            // Only solve new frames, and not more often than LIMELIGHT_SOLVES_PER_SECOND
            boolean solveDue = now - lastSolveNanos >= 1_000_000_000.0 / LIMELIGHT_SOLVES_PER_SECOND;
            if (solveDue && limelight.isNewFrame()) {
                lastSolveNanos = now;
                limelightPose = limelightPosition.getPedroPose();
                latency = limelight.getLatencySeconds();
            }
        }

        if (relocalizing && limelightPose != null) {
            addRelocalizeSample(limelightPose);
            // The old estimate is not trusted while relocalizing, the Limelight samples replace it
            limelightPose = null;
        }
        KalmanPositionFilter.FieldPosition position = positionFilter.update(
                pinpointX, pinpointY, pinpointHeading, limelightPose, latency, deltaTimeSeconds);
        if (!relocalizing && relocalizeCount > 0) {
            // A relocalize just finished, use the moved estimate this loop
            relocalizeCount = 0;
            position = positionFilter.getPosition();
        }
        updateVelocity(position.headingRadians, deltaTimeSeconds);
        return position;
    }

    // Collects how far the Limelight says the estimate is off, then moves the estimate by the average
    private void addRelocalizeSample(Pose limelightPose) {
        KalmanPositionFilter.FieldPosition estimate = positionFilter.getPosition();
        relocalizeSumX += limelightPose.x() - estimate.xInches;
        relocalizeSumY += limelightPose.y() - estimate.yInches;
        double headingOffset = limelightPose.heading() - estimate.headingRadians;
        relocalizeSumCos += Math.cos(headingOffset);
        relocalizeSumSin += Math.sin(headingOffset);
        relocalizeCount++;
        if (relocalizeCount >= RELOCALIZE_SAMPLES) {
            double headingShift = RELOCALIZE_HEADING ? Math.atan2(relocalizeSumSin, relocalizeSumCos) : 0;
            positionFilter.shiftEstimate(relocalizeSumX / relocalizeCount, relocalizeSumY / relocalizeCount, headingShift);
            relocalizing = false;
        }
    }

    // Pinpoint velocity turned into the field frame and smoothed, plus a smoothed acceleration
    private void updateVelocity(double fieldHeading, double deltaTimeSeconds) {
        double rotation = fieldHeading - pinpointHeading;
        double cos = Math.cos(rotation);
        double sin = Math.sin(rotation);
        double fieldVx = (pinpointVelX * cos - pinpointVelY * sin) / MM_PER_INCH;
        double fieldVy = (pinpointVelX * sin + pinpointVelY * cos) / MM_PER_INCH;
        // Share of the new reading to use, from this loop's real time, so a slow loop weighs in more
        double velocityBlend = getBlend(deltaTimeSeconds, VELOCITY_SMOOTHING_TIME);
        double accelBlend = getBlend(deltaTimeSeconds, ACCELERATION_SMOOTHING_TIME);
        double newVelX = velX + velocityBlend * (fieldVx - velX);
        double newVelY = velY + velocityBlend * (fieldVy - velY);
        if (deltaTimeSeconds > 1e-3) {
            accelX += accelBlend * ((newVelX - velX) / deltaTimeSeconds - accelX);
            accelY += accelBlend * ((newVelY - velY) / deltaTimeSeconds - accelY);
        }
        turnRate += velocityBlend * (pinpointTurnRate - turnRate);
        velX = newVelX;
        velY = newVelY;
    }

    // The first loop's time includes init, so it is skipped
    private void updateLoopTime(double deltaTimeSeconds, long now) {
        if (firstLoop) {
            firstLoop = false;
            maxLoopWindowStartNanos = now;
            return;
        }
        loopSeconds = deltaTimeSeconds;
        averageLoopSeconds = averageLoopSeconds == 0 ? loopSeconds : averageLoopSeconds + 0.05 * (loopSeconds - averageLoopSeconds);
        maxLoopSeconds = Math.max(maxLoopSeconds, loopSeconds);
        if (now - maxLoopWindowStartNanos >= 1_000_000_000L) {
            shownMaxLoopSeconds = maxLoopSeconds;
            maxLoopSeconds = 0;
            maxLoopWindowStartNanos = now;
        }
    }

    // 0 to 1, how much of a new reading to blend in after this much time for this smoothing time
    private static double getBlend(double deltaTimeSeconds, double smoothingTime) {
        if (smoothingTime <= 0) {
            return 1;
        }
        return 1 - Math.exp(-Math.max(0, deltaTimeSeconds) / smoothingTime);
    }

    // This loop's shot, null without a field position
    protected ShotCalculator.Shot getShot() {
        return shot;
    }

    // Shown on the Modes line
    protected String getRPMModeName() {
        return automaticRPM ? "auto" : "fixed";
    }

    // g2 dpad up = fixed RPM, down = automatic RPM. ShootTestButtonMap replaces this
    protected double getShooterRPM() {
        if (g2.dpadUpWasPressed()) {
            automaticRPM = false;
        }
        if (g2.dpadDownWasPressed()) {
            automaticRPM = true;
        }
        boolean shotValid = shot != null && shot.valid;
        return automaticRPM && shotValid ? shot.rpm : FIXED_RPM;
    }

    // ShootTestButtonMap uses g2's dpad left / right for the RPM, so it turns the hand count buttons off
    protected boolean ballCountButtonsEnabled() {
        return true;
    }

    public int getBallCount() {
        return ballCounter.getCount();
    }

    // Runs the ball detectors on this loop's readings (from last loop's powers) and updates the count
    private void updateBallCount(double requestedIntake, double targetRPM, double transferPower) {
        double now = System.nanoTime() / 1e9;
        boolean reversing = robot.intakeJamGuard != null && robot.intakeJamGuard.isReversing();
        boolean ballEntered = intakeBallDetector.update(lastIntakePower, robot.getIntakeVelocity(), reversing, now);
        int ballsShot = 0;
        if (COUNT_SHOTS) {
            // Only while the transfer feeds forward, atSpeed false also keeps the next dip from counting
            boolean feeding = transferPower > 0 && !robot.isShooterRamping();
            shotCounter.update(robot.getShootRPM(), targetRPM, now, feeding);
            int shots = shotCounter.getShots();
            ballsShot = shots - lastShots;
            lastShots = shots;
        }
        ballCounter.update(requestedIntake > INTAKE_HELD_POWER, ballEntered, ballsShot, ballCountButtonsEnabled());
        telemetry.addData("Balls", "%d%s%s", ballCounter.getCount(), ballCounter.isCapRemoved() ? " (no limit)" : "",
                ballCounter.isIntakeLocked() ? ", FULL, intake stopped (let go of intake to run it)" : "");
    }

    // {shooter RPM, intake power, transfer power, flower (1 = down)}, call after getPowers(), refills the same array
    public double[] getState() {
        double[] vals = state;
        vals[2] = 0;
        vals[3] = 0;
        vals[0] = getShooterRPM();

        // g1 right bumper shoots: transfer plus intake to push the balls through
        boolean shooting = g1.right_bumper;

        // g2's bumpers win over g1's shoot button
        if (g2.left_bumper) {
            vals[2] = -0.7;
        } else if (g2.right_bumper) {
            vals[2] = 0.7;
        } else if (shooting) {
            vals[2] = SHOOT_TRANSFER_POWER;
        }

        double requestedIntake = getIntakePower();
        updateBallCount(requestedIntake, vals[0], vals[2]);
        // Full from intaking: intaking stops, outtaking still works
        vals[1] = ballCounter.isIntakeLocked() && requestedIntake > 0 ? 0 : requestedIntake;
        // Balls pushed through by the shoot button aren't new balls, so only g2's intaking is watched
        lastIntakePower = vals[1];
        if (vals[1] == 0 && shooting) {
            vals[1] = SHOOT_INTAKE_POWER;
        }
        // Hold x to lower the flower
        if (g2.x) {
            vals[3] = 1;
        }
        return vals;
    }

    // g2's intake power sent last loop, the intake speed read this loop comes from it
    private double lastIntakePower = 0;
}
