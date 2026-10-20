package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BarebonesRobot;
import org.firstinspires.ftc.teamcode.ButtonMaps.BarebonesBM;
import org.firstinspires.ftc.teamcode.ButtonMaps.PPBM;
import org.firstinspires.ftc.teamcode.MotorPowers;
import org.firstinspires.ftc.teamcode.PPBot;

@TeleOp(name="TeleOpPP")
public class TeleOpPP extends OpMode {
    PPBot robot;
    PPBM bm;

    @Override
    public void init() {
        robot = new PPBot(hardwareMap);
        bm = new PPBM(this, robot);
    }

    @Override
    public void loop() {
        MotorPowers mp = bm.getPowers();
        robot.setPowers(mp);
        telemetry.addData("Motors", "left front(%.2f), right front(%.2f)", mp.lf, mp.rf);
        telemetry.addData("Motors", "left back(%.2f), right back(%.2f)", mp.lb, mp.rb);
        telemetry.addData("Pinpoint Angle", robot.pinpointAngle());
        telemetry.addData("Pinpoint X", robot.getPinpointX());
        telemetry.addData("Pinpoint Y", robot.getPinpointY());
    }
}
