package org.firstinspires.ftc.teamcode.ButtonMaps;

import static org.firstinspires.ftc.teamcode.ButtonMaps.BasicBM.getBumperRotateClockwise;
import static org.firstinspires.ftc.teamcode.ButtonMaps.BasicBM.getDpadForward;
import static org.firstinspires.ftc.teamcode.ButtonMaps.BasicBM.getDpadRight;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.BarebonesRobot;
import org.firstinspires.ftc.teamcode.MotorPowers;
import org.firstinspires.ftc.teamcode.PPBot;

public class PPBM extends BarebonesBM{
    private PPBot robot;
    public PPBM(OpMode op, PPBot r) {
        super(op, r);
        robot = r;
    }
    /**
     * Reads gamepad1's d-pad, bumpers, and both sticks and combines them into a single
     * drive command. Also refreshes Pinpoint odometry for this loop.
     *
     * @return per-wheel motor powers for the current gamepad1 input
     */
    public MotorPowers getPowers() {
        // Refresh odometry each loop before using any pinpoint-derived data.
        robot.pinpointUpdate();
        double forward = getDpadForward(g1.dpad_up, g1.dpad_down);
        double right = getDpadRight(g1.dpad_left, g1.dpad_right);
        double rotateClockwise = getBumperRotateClockwise(g1.left_bumper, g1.right_bumper);

        double[] stickMovement = getMovement(g1.left_stick_x, g1.left_stick_y);
        forward += stickMovement[0];
        right += stickMovement[1];

        rotateClockwise += getRotation(g1.right_stick_x);

        return Mechanumdrive.fieldOrientedDrive(forward, right, rotateClockwise, Math.PI/2+robot.pinpointAngle());

    }

    /**
     * Left stick translation: magnitude is deadzoned, rescaled to [0,1], then squared
     * for finer control at low speeds. Angle (from atan of y/x) determines direction.
     *
     * @param x left stick x value, [-1, 1]
     * @param y left stick y value, [-1, 1]
     * @return {forward, right} power components
     */
    public static double[] getMovement( double x, double y){
        y *= -1;
        double leftStickDistance = Math.sqrt(Math.pow(y, 2) + Math.pow(x, 2));
        if (leftStickDistance > 1.0) {
            leftStickDistance = 1.0;
        }
        if (leftStickDistance > LEFT_STICK_TRIGGER){
            double scaledDistance = leftStickDistance - LEFT_STICK_TRIGGER;
            scaledDistance /= 0.9;
            scaledDistance = Math.pow(scaledDistance, 2);
            double forward = y * scaledDistance / leftStickDistance;
            double right = x * scaledDistance / leftStickDistance;

            return new double[]{forward, right};
        }
        return new double[]{0, 0};
    }
    /**
     * Right stick x controls rotation, using the same deadzone/rescale/square curve as translation.
     *
     * @param rightx right stick x value, [-1, 1]
     * @return clockwise rotation power, [-1, 1]
     */
    public static double getRotation(double rightx){
        if (rightx > RIGHT_STICK_TRIGGER || rightx < -RIGHT_STICK_TRIGGER){
            double distance = Math.abs(rightx) - RIGHT_STICK_TRIGGER;
            distance /= 0.9;
            distance = Math.pow(distance, 2);
            if (rightx < 0){
                distance *= -1;
            }
            return distance;
        }

        return 0;
    }
}
