package org.firstinspires.ftc.teamcode.ButtonMaps;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Limelight.Limelight;
import org.firstinspires.ftc.teamcode.Limelight.LimelightBlue;
import org.firstinspires.ftc.teamcode.Limelight.calculatePosition;
import org.firstinspires.ftc.teamcode.Localization.KalmanPositionFilter;
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

    public WorkingBotButtonMap(OpMode op, Robot r) {
        super(op, r);
        robot = r;
        telemetry = op.telemetry;
        limelight = new Limelight(r);
        limelight.useAprilTagPipeline();
        limelightBlue = new LimelightBlue(limelight, telemetry);
    }

    /**
     * Gamepad 2 A turns to the shooting target and B turns to the nearest flower.
     * Moving gamepad 1's right stick cancels the turn so the driver always has control.
     *
     * @return clockwise rotation power, [-1, 1]
     */
    @Override
    protected double getRotateClockwise(){
        KalmanPositionFilter.FieldPosition position = updatePosition();
        if (g2.aWasPressed()){
            rotateToShootTarget(position);
        }
        if (g2.bWasPressed()){
            rotateToNearestFlower(position);
        }
        telemetry.addData("Rotate", "field pose known(%b), rotating(%b), target(%.1f deg)",
                positionFilter.hasFieldPose(), rotating, Math.toDegrees(rotateTargetHeading));

        double stickRotation = super.getRotateClockwise();
        if (stickRotation != 0){
            rotating = false;
            return stickRotation;
        }
        return getRotationPower(position.headingRadians);
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

    // Does nothing if the target can't be aimed at, aimState() needs a raised friendly CELL in view
    private void rotateToShootTarget(KalmanPositionFilter.FieldPosition position){
        if (!positionFilter.hasFieldPose()){
            return;
        }
        double[] aim = limelightBlue.aimState(new double[] {
                (position.xInches - FIELD_CENTER_INCHES) * METERS_PER_INCH,
                (position.yInches - FIELD_CENTER_INCHES) * METERS_PER_INCH,
                position.headingRadians});
        if (aim != null){
            rotateTo(position.headingRadians + aim[2]);
        }
    }

    private void rotateToNearestFlower(KalmanPositionFilter.FieldPosition position){
        if (!positionFilter.hasFieldPose()){
            return;
        }
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
