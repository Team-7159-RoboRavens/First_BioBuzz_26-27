package org.firstinspires.ftc.teamcode.ButtonMaps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.MotorPowers;
import org.firstinspires.ftc.teamcode.Robot;

public class BasicBM {
    private OpMode opmode;
    private Robot robot;
    private Gamepad g1;
    private Gamepad g2;
    public BasicBM(OpMode op, Robot r) {
        opmode = op;
        robot = r;
        g1 = opmode.gamepad1;
        g2 = opmode.gamepad2;
    }
    public MotorPowers getPowers() {
        double lf = 0;
        double rf = 0;
        double lb = 0;
        double rb = 0;
        if (g1.dpad_up) {
            lf++;
            rf++;
            lb++;
            rb++;
        } else if (g1.dpad_down) {
            lf--;
            rf--;
            lb--;
            rb--;
        }
        if (g1.dpad_left) {
            lf++;
            rf--;
            lb--;
            rb++;
        } else if (g1.dpad_right) {
            lf--;
            rf++;
            lb++;
            rb--;
        }

        if (g1.left_bumper) {
            lf++;
            rf--;
            lb++;
            rf--;
        } else if (g1.right_bumper) {
            lf--;
            rf++;
            lb--;
            rb++;
        }

        lf /= 3;
        rf /= 3;
        lb /= 3;
        rb /= 3;
        return new MotorPowers(lf, rf, lb, rb);
    }
}
