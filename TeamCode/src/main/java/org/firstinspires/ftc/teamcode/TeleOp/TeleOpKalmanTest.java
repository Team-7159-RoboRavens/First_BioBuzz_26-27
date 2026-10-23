package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ButtonMaps.Mechanumdrive;
import org.firstinspires.ftc.teamcode.ButtonMaps.PinpointBM;
import org.firstinspires.ftc.teamcode.Localization.KalmanPositionFilter;
import org.firstinspires.ftc.teamcode.MotorPowers;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "TeleOpKalmanTest")
public class TeleOpKalmanTest extends OpMode {
    private Robot robot;
    private KalmanPositionFilter positionFilter;
    private long previousUpdateNanos;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, 3);
        positionFilter = new KalmanPositionFilter();
        previousUpdateNanos = System.nanoTime();
    }

    @Override
    public void loop() {
        for (LynxModule hub : robot.hubs) {
            hub.clearBulkCache();
        }
        robot.pinpointUpdate();

        long currentTimeNanos = System.nanoTime();
        double deltaTimeSeconds = (currentTimeNanos - previousUpdateNanos) / 1_000_000_000.0;
        previousUpdateNanos = currentTimeNanos;
        KalmanPositionFilter.FieldPosition position = positionFilter.update(
                robot.getPinpointX(),
                robot.getPinpointY(),
                robot.pinpointAngle(),
                deltaTimeSeconds);

        double[] movement = PinpointBM.getMovement(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double forward = movement[0] + PinpointBM.getDpadForward(gamepad1.dpad_up, gamepad1.dpad_down);
        double right = movement[1] + PinpointBM.getDpadRight(gamepad1.dpad_left, gamepad1.dpad_right);
        double rotation = PinpointBM.getRotation(gamepad1.right_stick_x)
                + PinpointBM.getBumperRotateClockwise(gamepad1.left_bumper, gamepad1.right_bumper);

        MotorPowers powers = Mechanumdrive.fieldOrientedDrive(
                forward,
                right,
                rotation,
                Math.PI / 2.0 + position.headingRadians);
        robot.setPowers(powers);

        telemetry.addData("Estimated field X (mm)", "%.1f", position.xMillimeters);
        telemetry.addData("Estimated field Y (mm)", "%.1f", position.yMillimeters);
        telemetry.addData("Estimated heading (deg)", "%.1f", Math.toDegrees(position.headingRadians));
    }
}
