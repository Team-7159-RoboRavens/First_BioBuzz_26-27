package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ButtonMaps.BasicBM;
import org.firstinspires.ftc.teamcode.MotorPowers;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name="TeleOpBase")
public class TeleOpBase extends OpMode {
    Robot robot;
    BasicBM bm;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        bm = new BasicBM(this, robot);
    }

    @Override
    public void loop() {
        MotorPowers mp = bm.getPowers();
        robot.setPowers(mp);
        telemetry.addData("Motors", "left front(%.2f), right front(%.2f)", mp.lf, mp.rf);
        telemetry.addData("Motors", "left back(%.2f), right back(%.2f)", mp.lb, mp.rb);
    }
}
