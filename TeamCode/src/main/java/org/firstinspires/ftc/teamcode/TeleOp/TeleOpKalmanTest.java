package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ButtonMaps.Mechanumdrive;
import org.firstinspires.ftc.teamcode.ButtonMaps.PinpointBM;
import org.firstinspires.ftc.teamcode.Limelight.Limelight;
import org.firstinspires.ftc.teamcode.Limelight.LimelightBlue;
import org.firstinspires.ftc.teamcode.Limelight.calculatePosition;
import org.firstinspires.ftc.teamcode.Localization.KalmanPositionFilter;
import org.firstinspires.ftc.teamcode.MotorPowers;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "TeleOpKalmanTest")
public class TeleOpKalmanTest extends OpMode {
    private Robot robot;
    private Limelight limelight;
    // getPosition() uses every HIVE tag, so either alliance's class gives the same pose
    private calculatePosition limelightPosition;
    private KalmanPositionFilter positionFilter;
    private long previousUpdateNanos;
    private double previousLimelightFrameTimestamp = Double.NaN;
    // Last pose the Limelight produced, kept so it can be compared with the filter between sightings
    private double[] lastLimelightPosition;
    private Pose lastLimelightPedroPose;
    private long lastLimelightPoseNanos;


    PinpointBM bm;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, 0);
        limelight = new Limelight(robot);
        limelight.useAprilTagPipeline();
        limelightPosition = new LimelightBlue(limelight, telemetry);
        positionFilter = new KalmanPositionFilter();
        previousUpdateNanos = System.nanoTime();
        bm = new PinpointBM(this, robot);
    }

    @Override
    public void loop() {
        for (LynxModule hub : robot.hubs) {
            hub.clearBulkCache();
        }
        robot.pinpointUpdate();
        limelight.update();

        MotorPowers mp = bm.getPowers();
        robot.setPowers(mp);

        // Only use each Limelight frame once, and only when calculatePosition could turn it into a pose.
        // Limelight3A re-polls at 100 Hz, so the same camera frame comes back with new Control Hub timestamps;
        // the Limelight's own frame timestamp is what identifies a new image.
        Pose limelightPose = null;
        double limelightLatencySeconds = 0.0;
        LLResult result = limelight.latestResult;
        if (result != null && result.isValid() && result.getTimestamp() != previousLimelightFrameTimestamp) {
            previousLimelightFrameTimestamp = result.getTimestamp();
            double[] limelightFieldPosition = limelightPosition.getPosition();
            limelightPose = calculatePosition.toPedroPose(limelightFieldPosition);
            if (limelightPose != null) {
                lastLimelightPosition = limelightFieldPosition;
                lastLimelightPedroPose = limelightPose;
                lastLimelightPoseNanos = System.nanoTime();
            }
            limelightLatencySeconds =
                    (result.getCaptureLatency() + result.getTargetingLatency() + result.getStaleness()) / 1000.0;
        }

        long currentTimeNanos = System.nanoTime();
        double deltaTimeSeconds = (currentTimeNanos - previousUpdateNanos) / 1_000_000_000.0;
        previousUpdateNanos = currentTimeNanos;
        KalmanPositionFilter.FieldPosition position = positionFilter.update(
                robot.getPinpointX(),
                robot.getPinpointY(),
                robot.pinpointAngle(),
                limelightPose,
                limelightLatencySeconds,
                deltaTimeSeconds);

        double[] movement = PinpointBM.getMovement(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double forward = movement[0] + PinpointBM.getDpadForward(gamepad1.dpad_up, gamepad1.dpad_down);
        double right = movement[1] + PinpointBM.getDpadRight(gamepad1.dpad_left, gamepad1.dpad_right);
        double rotation = PinpointBM.getRotation(gamepad1.right_stick_x)
                + PinpointBM.getBumperRotateClockwise(gamepad1.left_bumper, gamepad1.right_bumper);

        // Drive on the fused heading so the Kalman estimate can be checked by feel
        MotorPowers powers = Mechanumdrive.fieldOrientedDrive(
                forward,
                right,
                rotation,
                Math.PI / 2.0 + position.headingRadians);
        robot.setPowers(powers);

        telemetry.addData("Field pose known", positionFilter.hasFieldPose());
        telemetry.addData("Limelight pose this loop", limelightPose != null);
        telemetry.addData("Kalman Pedro pose", "x: %.1f in, y: %.1f in, heading: %.1f deg",
                position.xInches, position.yInches, Math.toDegrees(position.headingRadians));
        if (lastLimelightPedroPose == null) {
            telemetry.addData("Limelight pose", "not detected yet");
            telemetry.addData("Limelight Pedro pose", "not detected yet");
        } else {
            telemetry.addData("Limelight pose age (s)", "%.2f", (currentTimeNanos - lastLimelightPoseNanos) / 1_000_000_000.0);
            telemetry.addData("Limelight pose", "x: %.3f m, y: %.3f m, heading: %.1f deg",
                    lastLimelightPosition[0], lastLimelightPosition[1], Math.toDegrees(lastLimelightPosition[2]));
            telemetry.addData("Limelight Pedro pose", "x: %.1f in, y: %.1f in, heading: %.1f deg",
                    lastLimelightPedroPose.x(), lastLimelightPedroPose.y(),
                    Math.toDegrees(lastLimelightPedroPose.heading()));
            double headingDifference = Math.IEEEremainder(position.headingRadians - lastLimelightPedroPose.heading(), 2.0 * Math.PI);
            telemetry.addData("Kalman - Limelight", "x: %.1f in, y: %.1f in, heading: %.1f deg",
                    position.xInches - lastLimelightPedroPose.x(),
                    position.yInches - lastLimelightPedroPose.y(),
                    Math.toDegrees(headingDifference));
        }
    }
}
