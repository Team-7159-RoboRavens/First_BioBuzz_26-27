package org.firstinspires.ftc.teamcode.ButtonMaps;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.MotorPowers;
import org.firstinspires.ftc.teamcode.Robot;

public class PinpointBM {
    private static final double LEFT_STICK_TRIGGER = 0.1;
    private static final double RIGHT_STICK_TRIGGER = 0.1;
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
        double power = 0;
        if (g1.right_trigger> 0.1){
            power = g1.right_trigger;
            power -= 0.1;
            power /= 0.9;
        }
        return power;
    }
    public MotorPowers getPowers() {
        robot.pinpointUpdate();
        double forward = 0;
        double right = 0;
        double rotateClockwise = 0;
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

        if (g1.left_bumper) {
            rotateClockwise --;
        } else if (g1.right_bumper) {
            rotateClockwise ++;
        }
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
        if (g1.right_stick_x > RIGHT_STICK_TRIGGER || g1.right_stick_x < -RIGHT_STICK_TRIGGER){
            double distance = g1.right_stick_x - LEFT_STICK_TRIGGER;
            distance /= 0.9;
            distance = Math.pow(distance, 2);
            if (g1.right_stick_x < 0){
                distance *= -1;
            }
            rotateClockwise += distance;
        }


        return Mechanumdrive.setMotorPowers(forward, right, rotateClockwise);

    }
    public static double getRotation(double rightx){
        if (rightx > RIGHT_STICK_TRIGGER || rightx < -RIGHT_STICK_TRIGGER){
            double distance = rightx - LEFT_STICK_TRIGGER;
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
