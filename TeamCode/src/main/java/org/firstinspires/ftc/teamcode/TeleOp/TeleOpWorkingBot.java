package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ButtonMaps.WheelTestBM;
import org.firstinspires.ftc.teamcode.ButtonMaps.WorkingBotButtonMap;
import org.firstinspires.ftc.teamcode.Limelight.Limelight;
import org.firstinspires.ftc.teamcode.MotorPowers;
import org.firstinspires.ftc.teamcode.Robot;
import com.qualcomm.hardware.lynx.LynxModule;
import org.firstinspires.ftc.teamcode.ButtonMaps.BasicBM;

@TeleOp(name = "TeleOpWorkingBot")
public class TeleOpWorkingBot extends OpMode{
    Robot robot;
    WorkingBotButtonMap bm;
    private ElapsedTime lastFrameTime = new ElapsedTime();

    public double averageFrameTime = 0;
    private long frameCount;
    Limelight ll;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, 10);
        bm = new WorkingBotButtonMap(this, robot);
    }

    @Override
    public void loop() {
        for (LynxModule hub : robot.hubs) {
            hub.clearBulkCache();
        }
        MotorPowers mp = bm.getPowers();
        robot.setPowers(mp);
        double[] vals = bm.getState();
        robot.setShootRPM(vals[0]);
        robot.setIntakePower(vals[1]);
        robot.setShootFeedPower(vals[2]);
        robot.setFlowerState(vals[3]);




        averageFrameTime = averageFrameTime * frameCount + lastFrameTime.milliseconds();
        averageFrameTime /= frameCount + 1;
        lastFrameTime.reset();
        frameCount ++;
        TeleOpData.frameTime = averageFrameTime;
        telemetry.addData("Motors", "shoot motor target(%.2f), actual(%.2f)",vals[0], robot.shootMotor.getVelocity() * 60 /386.3);
        telemetry.addData("Motors", "left front(%.2f), right front(%.2f)", mp.lf, mp.rf);
        telemetry.addData("Motors", "left back(%.2f), right back(%.2f)", mp.lb, mp.rb);
        telemetry.addData("Motors", "enabled(%b)", robot.inputHandlingEnabled);
    }
}
