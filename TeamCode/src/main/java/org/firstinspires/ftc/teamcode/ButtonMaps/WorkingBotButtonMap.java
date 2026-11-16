package org.firstinspires.ftc.teamcode.ButtonMaps;

import com.pedropathing.math.Pose;
// Limelight removed, the phone kept disconnecting from the driver station with it plugged in
// import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
// import org.firstinspires.ftc.teamcode.Limelight.Limelight;
import org.firstinspires.ftc.teamcode.Limelight.LimelightBlue;
import org.firstinspires.ftc.teamcode.Localization.KalmanPositionFilter;
import org.firstinspires.ftc.teamcode.MotorPowers;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOpData;

public class WorkingBotButtonMap extends PinpointBM{
    private static final double METERS_PER_INCH = 0.0254;
    // Pedro's origin is the corner of the field, the center of the field is at (72, 72) inches
    private static final double FIELD_CENTER_INCHES = 72;

    // Starting values for the automatic rotation, tune these on the robot
    private static final double ROTATE_KP = 1.0;
    private static final double ROTATE_MAX_POWER = 0.7;
    private static final double ROTATE_MIN_POWER = 0.1;
    private static final double ROTATE_TOLERANCE = Math.toRadians(2);
    // Starting values for driving to the shooting distance, tune these on the robot
    private static final double DRIVE_KP = 0.05; // power per inch of error
    private static final double DRIVE_MAX_POWER = 0.6;
    private static final double DRIVE_MIN_POWER = 0.12;
    private static final double DRIVE_TOLERANCE_INCHES = 1;

    private double RPM = 0;
    private final Robot robot;
    private final Telemetry telemetry;
    // private final Limelight limelight;
    // Only used for the shooting math, it has no Limelight
    private final LimelightBlue limelightBlue;
    // Pinpoint only, starts from where the auto ended
    private final KalmanPositionFilter positionFilter;
    private long previousUpdateNanos = System.nanoTime();
    // Last finite Pinpoint reading (mm, mm, radians)
    private double pinpointX = 0;
    private double pinpointY = 0;
    private double pinpointHeading = 0;
    // The Pinpoint takes a moment to reset and recalibrate, its jump back to 0 isn't the robot moving
    private static final long PINPOINT_RESET_NANOS = 400_000_000L;
    private long pinpointResetEndNanos = 0;
    // private double previousLimelightFrameTimestamp = Double.NaN;

    // Field heading the robot is turning to, in radians. A new call to rotateTo() replaces it.
    private double rotateTargetHeading = 0;
    private boolean rotating = false;
    // Pedro position (inches) the robot is driving to for the shot, only used while driving is true
    private double driveTargetX = 0;
    private double driveTargetY = 0;
    private boolean driving = false;

    public WorkingBotButtonMap(OpMode op, Robot r) {
        super(op, r);
        robot = r;
        telemetry = op.telemetry;
        // limelight = new Limelight(r);
        // limelight.useAprilTagPipeline();
        limelightBlue = new LimelightBlue(null, telemetry);
        // Without an auto pose the field position is unknown and A / B do nothing
        Pose autoEndPose = TeleOpData.autoEndPose;
        positionFilter = autoEndPose != null ? new KalmanPositionFilter(autoEndPose) : new KalmanPositionFilter();
        // Used once, a later teleop run starts somewhere else
        TeleOpData.autoEndPose = null;
    }

    /**
     * Gamepad 1 A drives to the shooting distance and turns so the shooter faces the target.
     * Any gamepad 1 movement (sticks, d-pad, bumpers) cancels it so the driver always has control.
     * Gamepad 1 B turns to the nearest flower; only gamepad 1 rotation cancels that turn.
     *
     * @return per-wheel motor powers
     */
    @Override
    public MotorPowers getPowers(){
        // Refresh odometry each loop before using any pinpoint-derived data.
        robot.pinpointUpdate();
        readPinpoint();
        KalmanPositionFilter.FieldPosition position = updatePosition();

        double forward = getDpadForward(g1.dpad_up, g1.dpad_down);
        double right = getDpadRight(g1.dpad_left, g1.dpad_right);
        double[] stickMovement = getMovement(g1.left_stick_x, g1.left_stick_y);
        forward += stickMovement[0];
        right += stickMovement[1];
        double rotateClockwise = getBumperRotateClockwise(g1.left_bumper, g1.right_bumper)
                + getRotation(g1.right_stick_x);

        if (g1.aWasPressed()){
            driveToShootPose(position);
        }
        if (g1.bWasPressed()){
            rotateToNearestFlower(position);
        }

        boolean driverTranslating = forward != 0 || right != 0;
        boolean driverRotating = rotateClockwise != 0;
        if (driving && (driverTranslating || driverRotating)){
            driving = false;
            rotating = false;
        }
        if (driverRotating){
            rotating = false;
        }

        telemetry.addData("Auto aim", "field pose known(%b), driving(%b), rotating(%b)",
                positionFilter.hasFieldPose(), driving, rotating);
        telemetry.addData("Auto aim", "target x(%.1f in), y(%.1f in), heading(%.1f deg)",
                driveTargetX, driveTargetY, Math.toDegrees(rotateTargetHeading));

        if (driving){
            double[] drivePower = getDrivePower(position);
            return Mechanumdrive.setMotorPowers(drivePower[0], drivePower[1],
                    getRotationPower(position.headingRadians));
        }
        if (!driverRotating){
            rotateClockwise = getRotationPower(position.headingRadians);
        }
        return Mechanumdrive.fieldOrientedDrive(forward, right, rotateClockwise, Math.PI/2 + pinpointHeading);
    }

    // Keeps the last good reading, a NaN from the Pinpoint would throw in the filter and stop the op mode
    private void readPinpoint(){
        double x = robot.getPinpointX();
        double y = robot.getPinpointY();
        double heading = robot.pinpointAngle();
        if (Double.isFinite(x) && Double.isFinite(y) && Double.isFinite(heading)){
            pinpointX = x;
            pinpointY = y;
            pinpointHeading = heading;
        }
    }

    // Runs the filter on this loop's Pinpoint reading, no Limelight
    private KalmanPositionFilter.FieldPosition updatePosition(){
        // limelight.update();
        // limelightBlue.updateTagPosition();

        // Pose limelightPose = null;
        // double limelightLatencySeconds = 0;
        // LLResult result = limelight.latestResult;
        // if (result != null && result.isValid() && result.getTimestamp() != previousLimelightFrameTimestamp){
        //     previousLimelightFrameTimestamp = result.getTimestamp();
        //     limelightPose = limelightBlue.getPedroPose();
        //     limelightLatencySeconds =
        //             (result.getCaptureLatency() + result.getTargetingLatency() + result.getStaleness()) / 1000.0;
        // }

        long currentTimeNanos = System.nanoTime();
        double deltaTimeSeconds = (currentTimeNanos - previousUpdateNanos) / 1_000_000_000.0;
        previousUpdateNanos = currentTimeNanos;
        if (currentTimeNanos < pinpointResetEndNanos){
            positionFilter.resetPinpointReference();
        }
        return positionFilter.update(
                pinpointX,
                pinpointY,
                pinpointHeading,
                deltaTimeSeconds);
    }

    // Kalman (Pinpoint + Limelight) pose in the meters / field center frame calculatePosition uses
    private static double[] toFieldMeters(KalmanPositionFilter.FieldPosition position){
        return new double[] {
                (position.xInches - FIELD_CENTER_INCHES) * METERS_PER_INCH,
                (position.yInches - FIELD_CENTER_INCHES) * METERS_PER_INCH,
                position.headingRadians};
    }

    // Does nothing if the target can't be aimed at. Without the Limelight it can't check that the CELL is raised.
    private void driveToShootPose(KalmanPositionFilter.FieldPosition position){
        if (!positionFilter.hasFieldPose()){
            return;
        }
        double[] shootPose = limelightBlue.getShootPose(toFieldMeters(position), false);
        if (shootPose == null){
            return;
        }
        driving = true;
        setShootTarget(shootPose);
    }

    private void setShootTarget(double[] shootPose){
        driveTargetX = shootPose[0] / METERS_PER_INCH + FIELD_CENTER_INCHES;
        driveTargetY = shootPose[1] / METERS_PER_INCH + FIELD_CENTER_INCHES;
        rotateTo(shootPose[2]);
    }

    /**
     * Re-solves the shooting pose from this loop's Kalman pose, so the distance to the target always comes from
     * the Limelight-corrected position (not just from where the robot was when A was pressed), then drives toward
     * it with proportional control, turned into robot-relative powers using the filtered heading.
     *
     * @return {forward, right} robot-relative power, {0, 0} once the robot is at the target
     */
    private double[] getDrivePower(KalmanPositionFilter.FieldPosition position){
        double[] botPose = toFieldMeters(position);
        double[] shootPose = limelightBlue.getShootPose(botPose, false);
        if (shootPose == null){
            driving = false;
            rotating = false;
            return new double[] {0, 0};
        }
        setShootTarget(shootPose);
        telemetry.addData("Auto aim", "shooter distance(%.1f in), wanted(%.1f in)",
                limelightBlue.getShooterDistance(botPose) / METERS_PER_INCH,
                limelightBlue.getWantedShooterDistance(botPose) / METERS_PER_INCH);

        double errorX = driveTargetX - position.xInches;
        double errorY = driveTargetY - position.yInches;
        double distance = Math.hypot(errorX, errorY);
        if (distance < DRIVE_TOLERANCE_INCHES){
            double headingError = Math.IEEEremainder(rotateTargetHeading - position.headingRadians, 2 * Math.PI);
            if (Math.abs(headingError) < ROTATE_TOLERANCE){
                driving = false;
                rotating = false;
            }
            return new double[] {0, 0};
        }
        double power = Math.max(DRIVE_MIN_POWER, Math.min(DRIVE_MAX_POWER, distance * DRIVE_KP));
        double fieldX = errorX / distance * power;
        double fieldY = errorY / distance * power;
        double cos = Math.cos(position.headingRadians);
        double sin = Math.sin(position.headingRadians);
        double forward = fieldX * cos + fieldY * sin;
        double right = fieldX * sin - fieldY * cos;
        return new double[] {forward, right};
    }

    private void rotateToNearestFlower(KalmanPositionFilter.FieldPosition position){
        if (!positionFilter.hasFieldPose()){
            return;
        }
        driving = false;
        rotateTo(getNearestFlowerHeading(
                position.xInches - FIELD_CENTER_INCHES,
                position.yInches - FIELD_CENTER_INCHES));
    }

    /**
     * The lines y = x and y = -x split the field into 4 quadrants around the +X, +Y, -X and -Y axes,
     * with one flower per quadrant.
     *
     * @param x robot x relative to the center of the field
     * @param y robot y relative to the center of the field
     * @return field heading facing the flower of the quadrant the robot is in, in radians
     */
    public static double getNearestFlowerHeading(double x, double y){
        if (Math.abs(x) > Math.abs(y)){
            return x > 0 ? 0 : Math.PI;
        }
        return y > 0 ? Math.PI / 2 : -Math.PI / 2;
    }

    /**
     * Starts turning to a field heading. If a turn is already running the robot goes to this new heading instead.
     *
     * @param heading field heading to turn to, in radians
     */
    private void rotateTo(double heading){
        rotateTargetHeading = heading;
        rotating = true;
    }

    /**
     * Proportional control toward rotateTargetHeading. Headings grow counterclockwise and
     * rotation power is clockwise, so the sign flips.
     *
     * @param heading current field heading, in radians
     * @return clockwise rotation power, 0 when no turn is running
     */
    private double getRotationPower(double heading){
        if (!rotating){
            return 0;
        }
        double error = Math.IEEEremainder(rotateTargetHeading - heading, 2 * Math.PI);
        if (Math.abs(error) < ROTATE_TOLERANCE){
            rotating = false;
            return 0;
        }
        double power = Math.max(ROTATE_MIN_POWER, Math.min(ROTATE_MAX_POWER, Math.abs(error) * ROTATE_KP));
        return error > 0 ? -power : power;
    }


    //return a 3 value array where the first value is rpm of the shoot motor the
    // second is intake the third is feed motor and the 4th is the thing for getting stuff out of flowers
    public double[] getState(){
        double[] vals = new double[4];
        vals[1] = getIntakePower();
        if (g2.dpadUpWasPressed()){
            RPM = 183;
        }
        if (g2.dpadDownWasPressed()){
            RPM = 0;
        }
        if (g2.dpadRightWasPressed()){
            RPM += 5;
        }
        if (g2.dpadLeftWasPressed()){
            RPM -= 5;
        }
        // Feed only, the intake stays on the triggers
        if (g2.left_bumper){
            vals[2] = -0.7;
        }
        else if (g2.right_bumper){
            vals[2] = 0.7;
        }
        vals[0] = RPM;
        // Hold x to lower the flower
        if (g2.x){
            vals[3] = 1;
        }
        if (g1.backWasPressed()){
            robot.resetIMU();
            pinpointResetEndNanos = System.nanoTime() + PINPOINT_RESET_NANOS;
        }
        return vals;
    }
}
