package org.firstinspires.ftc.teamcode.ButtonMaps;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.MotorPowers;
import org.firstinspires.ftc.teamcode.Robot;

// Button map used with the GoBilda Pinpoint odometry computer.
// Input math is split into static helpers (vs. BasicBM) so they can be unit tested directly.
public class PinpointBM {
    // Deadzone thresholds below which stick input is ignored.
    public static final double LEFT_STICK_TRIGGER = 0.1;
    public static final double RIGHT_STICK_TRIGGER = 0.1;
    private OpMode opmode;
    private Robot robot;
    private Gamepad g1;
    private Gamepad g2;
    public PinpointBM(OpMode op, Robot r) {
        opmode = op;
        robot = r;
        g1 = opmode.gamepad1;
        g2 = opmode.gamepad2;
    }
    public double getIntakePower(){
        return getIntakePower(g1.right_trigger);
    }

    // Rescales the trigger from [0.1, 1] to [0, 1] so power ramps up smoothly past the deadzone.
    public static double getIntakePower(double rightTrigger){
        double power = 0;
        if (rightTrigger > 0.1){
            power = rightTrigger;
            power -= 0.1;
            power /= 0.9;
        }
        return power;
    }

    // D-pad up/down gives full-speed digital forward input.
    public static double getDpadForward(boolean dpadUp, boolean dpadDown){
        double forward = 0;
        if (dpadUp) {
            forward ++;
        } else if (dpadDown) {
            forward --;
        }
        return forward;
    }

    // D-pad left/right gives full-speed digital strafe input.
    public static double getDpadRight(boolean dpadLeft, boolean dpadRight){
        double right = 0;
        if (dpadLeft) {
            right --;
        } else if (dpadRight) {
            right ++;
        }
        return right;
    }

    // Bumpers give full-speed digital rotation.
    public static double getBumperRotateClockwise(boolean leftBumper, boolean rightBumper){
        double rotateClockwise = 0;
        if (leftBumper) {
            rotateClockwise --;
        } else if (rightBumper) {
            rotateClockwise ++;
        }
        return rotateClockwise;
    }

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

        return Mechanumdrive.setMotorPowers(forward, right, rotateClockwise);

    }

    // Left stick translation: magnitude is deadzoned, rescaled to [0,1], then squared
    // for finer control at low speeds. Angle (from atan of y/x) determines direction.
    // Returns {forward, right} components.
    public static double[] getMovement( double x, double y){
        double leftStickDistance = Math.sqrt(Math.pow(y, 2) + Math.pow(x, 2));
        if (leftStickDistance > LEFT_STICK_TRIGGER){
            leftStickDistance -= LEFT_STICK_TRIGGER;
            leftStickDistance /= 0.9;
            leftStickDistance = Math.pow(leftStickDistance, 2);
            double angle = Math.atan(-y/x);
            if (x < 0){
                angle += Math.PI;
            }

            return new double[]{Math.sin(angle) * leftStickDistance, Math.cos(angle) * leftStickDistance};
        }
        return new double[]{0, 0};
    }
    // Right stick x controls rotation, using the same deadzone/rescale/square curve as translation.
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
