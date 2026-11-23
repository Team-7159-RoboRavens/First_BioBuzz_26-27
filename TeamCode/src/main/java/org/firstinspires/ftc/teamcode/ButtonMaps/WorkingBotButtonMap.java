package org.firstinspires.ftc.teamcode.ButtonMaps;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Limelight.Limelight;
import org.firstinspires.ftc.teamcode.Limelight.LimelightBlue;
import org.firstinspires.ftc.teamcode.Localization.KalmanPositionFilter;
import org.firstinspires.ftc.teamcode.MotorPowers;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Shooter.ShotCalculator;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOpData;

import java.util.Locale;

// Gamepad 1: drive, hold LT to aim at our HIVE, hold RT to strafe along the shooting line while aiming,
//            A = turn the intake to face the nearest flower, hold right bumper = wiggle side to side to free a stuck
//            flower, back = relocalize from the Limelight
// Gamepad 2: triggers intake, bumpers transfer, x flower, back = make the current facing forward,
//            dpad up fixed RPM / down automatic RPM, dpad left automatic transfer on / right manual transfer only
public class WorkingBotButtonMap extends PinpointBM {
    // ---------------- Tuning: turning to aim ----------------
    // Turn power per radian of aim error. Higher = snappier aim, too high wobbles around the target
    public static double ROTATE_KP = 1.2;
    public static double ROTATE_MAX_POWER = 0.7;
    // Smallest turn power used outside the tolerance, so it doesn't stall just short of the target
    public static double ROTATE_MIN_POWER = 0.06;
    public static double ROTATE_TOLERANCE_DEG = 1;

    // ---------------- Tuning: automatic transfer ----------------
    // The transfer only runs on its own when the aim is within this many degrees...
    public static double AUTO_FEED_HEADING_TOLERANCE_DEG = 3;
    // ...and the flywheel is within this many RPM of its target
    public static double AUTO_FEED_RPM_TOLERANCE = 6;
    // Transfer and intake power while feeding on its own (same direction as the manual right bumper / right trigger)
    public static double AUTO_FEED_POWER = 0.7;
    public static double AUTO_FEED_INTAKE_POWER = 0.6;

    // ---------------- Tuning: strafing (RT) ----------------
    // The strafe line runs along x, this many inches from the field center on the robot's side (auto shoots from about 55)
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

    // ---------------- Tuning: wiggle (right bumper) ----------------
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
    // How much of each new reading goes into the smoothed speed / acceleration (1 = no smoothing, lower = smoother but slower)
    public static double VELOCITY_SMOOTHING = 0.5;
    public static double ACCELERATION_SMOOTHING = 0.2;

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

    // Last finite Pinpoint reading (mm, mm, radians, mm/s)
    private double pinpointX = 0;
    private double pinpointY = 0;
    private double pinpointHeading = 0;
    private double pinpointVelX = 0;
    private double pinpointVelY = 0;
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

    // A: field heading the intake is turning to
    private boolean flowerTurning = false;
    private double flowerTargetHeading = 0;

    // Right bumper wiggle: where it started, the robot's right at the start, and which side it is heading to
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
    private boolean automaticFeed = true;

    // Results of this loop's getPowers(), used by getState()
    private ShotCalculator.Shot shot = null;
    private boolean onTarget = false;

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
        double[] stickMovement = getMovement(g1.left_stick_x, g1.left_stick_y);
        forward += stickMovement[0];
        right += stickMovement[1];
        // Only the right stick turns, the bumpers don't
        double driverRotate = getRotation(g1.right_stick_x);
        boolean driverTranslating = forward != 0 || right != 0;
        boolean driverRotating = driverRotate != 0;

        boolean strafeHeld = g1.right_trigger > TRIGGER_PRESSED;
        boolean aimHeld = g1.left_trigger > TRIGGER_PRESSED;
        boolean aiming = (strafeHeld || aimHeld) && fieldPoseKnown;
        if (!strafeHeld || driverTranslating) {
            // Getting back onto the line first once the trigger is held again or the driver lets go
            strafeStage = StrafeStage.APPROACH;
        }

        shot = fieldPoseKnown ? ShotCalculator.solve(position.xInches, position.yInches, position.headingRadians,
                velX, velY, accelX, accelY, red) : null;
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
        onTarget = aiming && shotValid && !driverRotating && !flowerTurning
                && Math.abs(headingError) < Math.toRadians(AUTO_FEED_HEADING_TOLERANCE_DEG);

        telemetry.addData("Field pose", "known(%b) x(%.1f) y(%.1f) heading(%.1f deg)", fieldPoseKnown,
                position.xInches, position.yInches, Math.toDegrees(position.headingRadians));
        telemetry.addData("Shot", shotValid ? String.format(Locale.US, "dist %.1f in, rpm %.0f, aim err %.1f deg",
                shot.distance, shot.rpm, Math.toDegrees(headingError)) : "none");
        telemetry.addData("Modes", "rpm %s, transfer %s%s", automaticRPM ? "auto" : "fixed",
                automaticFeed ? "auto+manual" : "manual", relocalizing ? ", RELOCALIZING" : "");

        // Translation: the driver always wins, then the wiggle, otherwise strafe while RT is held
        boolean wiggleHeld = g1.right_bumper && !driverTranslating;
        if (!wiggleHeld) {
            wiggling = false;
        }
        double[] fieldPower = null;
        if (wiggleHeld) {
            fieldPower = getWiggleFieldPower(position);
        } else if (strafeHeld && fieldPoseKnown && !driverTranslating) {
            fieldPower = getStrafeFieldPower(position);
        }
        if (fieldPower != null) {
            double cos = Math.cos(position.headingRadians);
            double sin = Math.sin(position.headingRadians);
            double robotForward = fieldPower[0] * cos + fieldPower[1] * sin;
            double robotRight = fieldPower[0] * sin - fieldPower[1] * cos;
            return Mechanumdrive.setMotorPowers(robotForward, robotRight, rotate);
        }
        return Mechanumdrive.fieldOrientedDrive(forward, right, rotate, Math.PI / 2 + getDriveHeading(position));
    }

    // Robot heading measured from the driving forward direction, radians
    private double getDriveHeading(KalmanPositionFilter.FieldPosition position) {
        if (customForward) {
            return pinpointHeading - customForwardPinpointHeading;
        }
        if (positionFilter.hasFieldPose()) {
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

    // Field (x, y) power for the wiggle: right WIGGLE_DISTANCE from where it started, then back and forth
    // between WIGGLE_DISTANCE left and right of that start. Works without a field position, it only needs to move
    private double[] getWiggleFieldPower(KalmanPositionFilter.FieldPosition position) {
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
        return new double[] {wiggleRightX * power, wiggleRightY * power};
    }

    private double getAimRotationPower(double error) {
        if (Math.abs(error) < Math.toRadians(ROTATE_TOLERANCE_DEG)) {
            return 0;
        }
        double power = Math.max(ROTATE_MIN_POWER, Math.min(ROTATE_MAX_POWER, Math.abs(error) * ROTATE_KP));
        // Headings grow counterclockwise and rotation power is clockwise
        return error > 0 ? -power : power;
    }

    // Field (x, y) power: get onto the strafe line as fast as possible, then go back and forth along it
    private double[] getStrafeFieldPower(KalmanPositionFilter.FieldPosition position) {
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
                return new double[] {errorX / distance * power, errorY / distance * power};
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
        double holdLine = Math.max(-0.5, Math.min(0.5, (lineY - position.yInches) * LINE_KP));
        return new double[] {strafeDirection * STRAFE_POWER, holdLine};
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
    }

    // Runs the filter on this loop's Pinpoint reading plus a Limelight pose when there is a new one
    private KalmanPositionFilter.FieldPosition updatePosition() {
        long now = System.nanoTime();
        double deltaTimeSeconds = (now - previousUpdateNanos) / 1_000_000_000.0;
        previousUpdateNanos = now;

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
        double newVelX = velX + VELOCITY_SMOOTHING * (fieldVx - velX);
        double newVelY = velY + VELOCITY_SMOOTHING * (fieldVy - velY);
        if (deltaTimeSeconds > 1e-3) {
            accelX += ACCELERATION_SMOOTHING * ((newVelX - velX) / deltaTimeSeconds - accelX);
            accelY += ACCELERATION_SMOOTHING * ((newVelY - velY) / deltaTimeSeconds - accelY);
        }
        velX = newVelX;
        velY = newVelY;
    }

    // {shooter RPM, intake power, transfer power, flower (1 = down)}, call after getPowers()
    public double[] getState() {
        if (g2.dpadUpWasPressed()) {
            automaticRPM = false;
        }
        if (g2.dpadDownWasPressed()) {
            automaticRPM = true;
        }
        if (g2.dpadLeftWasPressed()) {
            automaticFeed = true;
        }
        if (g2.dpadRightWasPressed()) {
            automaticFeed = false;
        }

        double[] vals = new double[4];
        boolean shotValid = shot != null && shot.valid;
        vals[0] = automaticRPM && shotValid ? shot.rpm : FIXED_RPM;

        boolean autoFeeding = automaticFeed && onTarget
                && Math.abs(robot.getShootRPM() - vals[0]) < AUTO_FEED_RPM_TOLERANCE;
        telemetry.addData("Auto transfer", autoFeeding);

        vals[1] = getIntakePower();
        if (vals[1] == 0 && autoFeeding) {
            vals[1] = AUTO_FEED_INTAKE_POWER;
        }
        // Manual transfer always works and wins over the automatic one
        if (g2.left_bumper) {
            vals[2] = -0.7;
        } else if (g2.right_bumper) {
            vals[2] = 0.7;
        } else if (autoFeeding) {
            vals[2] = AUTO_FEED_POWER;
        }
        // Hold x to lower the flower
        if (g2.x) {
            vals[3] = 1;
        }
        return vals;
    }
}
