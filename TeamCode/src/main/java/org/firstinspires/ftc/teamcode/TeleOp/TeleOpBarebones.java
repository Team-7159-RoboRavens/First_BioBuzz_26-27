package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BarebonesRobot;
import org.firstinspires.ftc.teamcode.ButtonMaps.BarebonesBM;
import org.firstinspires.ftc.teamcode.MotorPowers;

// Drive only, field oriented from the Pinpoint heading, g1 back makes the current facing forward
@TeleOp(name = "TeleOpBarebones")
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
        telemetry.addData("Heading", "%.1f deg (g1 back = make this forward)", Math.toDegrees(robot.pinpointAngle()));
        telemetry.addData("Motors", "lf(%.2f) rf(%.2f) lb(%.2f) rb(%.2f)", mp.lf, mp.rf, mp.lb, mp.rb);
    }
}
