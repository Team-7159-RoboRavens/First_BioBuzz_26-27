package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ButtonMaps.BasicBM;
import org.firstinspires.ftc.teamcode.Limelight.Limelight;
import org.firstinspires.ftc.teamcode.MotorPowers;
import org.firstinspires.ftc.teamcode.Robot;
import com.qualcomm.hardware.lynx.LynxModule;

import com.qualcomm.robotcore.util.ElapsedTime;


// TeleOp op mode using the Pinpoint-aware button map, with odometry telemetry.

@TeleOp(name = "TeleOpStarterBot")
public class TeleOpStarterBot extends OpMode{
    Robot robot;
    BasicBM bm;
    private ElapsedTime lastFrameTime = new ElapsedTime();

    public double averageFrameTime = 0;
    private long frameCount;
    Limelight ll;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, 4);
        bm = new BasicBM(this, robot);
    }

    @Override
    public void loop() {
        for (LynxModule hub : robot.hubs) {
            hub.clearBulkCache();
        }
        MotorPowers mp = bm.getPowers();
        robot.setPowers(mp);
        robot.setIntakePower(bm.getIntakePower());




        averageFrameTime = averageFrameTime * frameCount + lastFrameTime.milliseconds();
        averageFrameTime /= frameCount + 1;
        lastFrameTime.reset();
        frameCount ++;
        TeleOpData.frameTime = averageFrameTime;
        telemetry.addData("Motors", "left front(%.2f), right front(%.2f)", mp.lf, mp.rf);
        telemetry.addData("Motors", "left back(%.2f), right back(%.2f)", mp.lb, mp.rb);
        telemetry.addData("Motors", "enabled(%b)", robot.inputHandlingEnabled);
    }
}
