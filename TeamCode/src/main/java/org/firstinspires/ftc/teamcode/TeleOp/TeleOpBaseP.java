package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ButtonMaps.BasicBM;
import org.firstinspires.ftc.teamcode.ButtonMaps.PinpointBM;
import org.firstinspires.ftc.teamcode.MotorPowers;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name="TeleOpBaseP")
public class TeleOpBaseP extends OpMode {
    Robot robot;
    PinpointBM pm;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        pm = new PinpointBM(this, robot);
    }

    @Override
    public void loop() {
        MotorPowers mp = pm.getPowers();
        robot.setPowers(mp);
        robot.setIntakePower(pm.getIntakePower());
        telemetry.addData("Motors", "left front(%.2f), right front(%.2f)", mp.lf, mp.rf);
        telemetry.addData("Motors", "left back(%.2f), right back(%.2f)", mp.lb, mp.rb);
        telemetry.addData("Pinpoint Angle", robot.pinpointAngle());
        telemetry.addData("Pinpoint X", robot.getPinpointX());
        telemetry.addData("Pinpoint Y", robot.getPinpointY());
    }
}
