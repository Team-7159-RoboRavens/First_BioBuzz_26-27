package org.firstinspires.ftc.teamcode.ButtonMaps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.BarebonesRobot;
import org.firstinspires.ftc.teamcode.MotorPowers;

// Basic button map: reads gamepad1 input and converts it into motor powers.
public class BarebonesBM {
    // Deadzone thresholds below which stick input is ignored.
    static final double LEFT_STICK_TRIGGER = 0.1;
    static final double RIGHT_STICK_TRIGGER = 0.1;
    OpMode opmode;
    BarebonesRobot robot;
    Gamepad g1;
    Gamepad g2;
    /**
     * @param op running op mode, used to read gamepad state
     * @param r robot instance this button map drives
     */
    public BarebonesBM(OpMode op, BarebonesRobot r) {
        opmode = op;
        robot = r;
        g1 = opmode.gamepad1;
        g2 = opmode.gamepad2;
    }
    /**
     * Rescales gamepad1's right trigger from [0.1, 1] to [0, 1] so power ramps up smoothly past the deadzone.
     *
     * @return intake motor power in [0, 1]
     */

    /**
     * Reads gamepad1's d-pad, bumpers, and both sticks and combines them into a single
     * drive command.
     *
     * @return per-wheel motor powers for the current gamepad1 input
     */
    public MotorPowers getPowers() {
        robot.pinpointUpdate();
        double forward = 0;
        double right = 0;
        double rotateClockwise = 0;
        // D-pad gives full-speed digital strafing/forward input.
        if (g1.dpad_up) {
            forward ++;
        } else if (g1.dpad_down) {
            forward --;
        }
        if (g1.dpad_left) {
            right --;
        } else if (g1.dpad_right) {
            right ++;
        }

        // Bumpers give full-speed digital rotation.
        if (g1.left_bumper) {
            rotateClockwise --;
        } else if (g1.right_bumper) {
            rotateClockwise ++;
        }
        // Left stick controls translation: magnitude is deadzoned, rescaled to [0,1],
        // then squared for finer control at low speeds. Angle determines direction.
        double leftStickDistance = Math.sqrt(Math.pow(g1.left_stick_y, 2) + Math.pow(g1.left_stick_x, 2));
        if (leftStickDistance > LEFT_STICK_TRIGGER){
            leftStickDistance -= LEFT_STICK_TRIGGER;
            leftStickDistance /= 0.9;
            leftStickDistance = Math.pow(leftStickDistance, 2);
            double angle = Math.atan(-g1.left_stick_y/g1.left_stick_x);
            if (g1.left_stick_x < 0){
                angle += Math.PI;
            }
            forward += Math.sin(angle) * leftStickDistance;
            right += Math.cos(angle) * leftStickDistance;
        }
        // Right stick x controls rotation, using the same deadzone/rescale/square curve.
        if (g1.right_stick_x > RIGHT_STICK_TRIGGER || g1.right_stick_x < -RIGHT_STICK_TRIGGER){
            double distance = g1.right_stick_x - LEFT_STICK_TRIGGER;
            distance /= 0.9;
            distance = Math.pow(distance, 2);
            if (g1.right_stick_x < 0){
                distance *= -1;
            }
            rotateClockwise += distance;
        }

        return Mechanumdrive.fieldOrientedDrive(forward, right, rotateClockwise,
            Math.PI / 2 + robot.pinpointAngle());
    }
}
