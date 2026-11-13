package org.firstinspires.ftc.teamcode.ButtonMaps;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Limelight.Limelight;
import org.firstinspires.ftc.teamcode.Limelight.LimelightBlue;
import org.firstinspires.ftc.teamcode.Limelight.calculatePosition;
import org.firstinspires.ftc.teamcode.Localization.KalmanPositionFilter;
import org.firstinspires.ftc.teamcode.MotorPowers;
import org.firstinspires.ftc.teamcode.Robot;

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
    private final Limelight limelight;
    private final LimelightBlue limelightBlue;
    private final KalmanPositionFilter positionFilter = new KalmanPositionFilter();
    private long previousUpdateNanos = System.nanoTime();
    private double previousLimelightFrameTimestamp = Double.NaN;

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
        limelight = new Limelight(r);
        limelight.useAprilTagPipeline();
        limelightBlue = new LimelightBlue(limelight, telemetry);
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
        return Mechanumdrive.fieldOrientedDrive(forward, right, rotateClockwise, Math.PI/2 + robot.pinpointAngle());
    }

    // Fuses this loop's Pinpoint reading with the Limelight, only using each Limelight frame once
    private KalmanPositionFilter.FieldPosition updatePosition(){
        limelight.update();
        limelightBlue.updateTagPosition();

        Pose limelightPose = null;
        double limelightLatencySeconds = 0;
        LLResult result = limelight.latestResult;
        if (result != null && result.isValid() && result.getTimestamp() != previousLimelightFrameTimestamp){
            previousLimelightFrameTimestamp = result.getTimestamp();
            limelightPose = limelightBlue.getPedroPose();
            limelightLatencySeconds =
                    (result.getCaptureLatency() + result.getTargetingLatency() + result.getStaleness()) / 1000.0;
        }

        long currentTimeNanos = System.nanoTime();
        double deltaTimeSeconds = (currentTimeNanos - previousUpdateNanos) / 1_000_000_000.0;
        previousUpdateNanos = currentTimeNanos;
        return positionFilter.update(
                robot.getPinpointX(),
                robot.getPinpointY(),
                robot.pinpointAngle(),
                limelightPose,
                limelightLatencySeconds,
                deltaTimeSeconds);
    }

    // Kalman (Pinpoint + Limelight) pose in the meters / field center frame calculatePosition uses
    private static double[] toFieldMeters(KalmanPositionFilter.FieldPosition position){
        return new double[] {
                (position.xInches - FIELD_CENTER_INCHES) * METERS_PER_INCH,
                (position.yInches - FIELD_CENTER_INCHES) * METERS_PER_INCH,
                position.headingRadians};
    }

    // Does nothing if the target can't be aimed at, getShootPose() needs a raised friendly CELL in view on the press.
    private void driveToShootPose(KalmanPositionFilter.FieldPosition position){
        if (!positionFilter.hasFieldPose()){
            return;
        }
        double[] shootPose = limelightBlue.getShootPose(toFieldMeters(position));
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
            RPM = 200;
        }
        if (g2.dpadDownWasPressed()){
            RPM = 0;
        }
        if (g2.dpadRightWasPressed()){
            RPM += 5;
        }
        if (g2.dpadLeftWasPressed()){
            RPM -= 5;
            if (RPM < 0){
                double feedPower = -0.7;
                vals[2] = feedPower;
            }
        }
        else if (g2.right_bumper){
            double feedPower = 0.7;
            vals[2] = feedPower;
            vals[1] = 0.5;
        }
        vals[0] = RPM;
        if (g2.left_bumper){
            vals[3] = 1;
        }
        return vals;
    }
}
