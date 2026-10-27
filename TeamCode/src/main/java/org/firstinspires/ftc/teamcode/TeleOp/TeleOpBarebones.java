package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BarebonesRobot;
import org.firstinspires.ftc.teamcode.ButtonMaps.BarebonesBM;
import org.firstinspires.ftc.teamcode.ButtonMaps.BasicBM;
import org.firstinspires.ftc.teamcode.MotorPowers;
import org.firstinspires.ftc.teamcode.Robot;

// TeleOp op mode using the basic (non-Pinpoint) button map.
@TeleOp(name="TeleOpBarebones")
public class TeleOpBarebones extends OpMode {
    BarebonesRobot robot;
    BarebonesBM bm;

    @Override
    public void init() {
        robot = new BarebonesRobot(hardwareMap);
        bm = new BarebonesBM(this, robot);
    }

    @Override
    public void loop() {
        MotorPowers mp = bm.getPowers();
        robot.setPowers(mp);
        telemetry.addData("Motors", "left front(%.2f), right front(%.2f)", mp.lf, mp.rf);
        telemetry.addData("Motors", "left back(%.2f), right back(%.2f)", mp.lb, mp.rb);
        telemetry.addData("Left stick X", "%.3f", gamepad1.left_stick_x);
        telemetry.addData("Pinpoint heading (rad)", "%.3f", robot.pinpointAngle());
    }
}
