package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.ButtonMaps.WheelTestBM;
import org.firstinspires.ftc.teamcode.ButtonMaps.WorkingBotButtonMap;
import org.firstinspires.ftc.teamcode.Limelight.Limelight;
import org.firstinspires.ftc.teamcode.Macros.GoToShoot;
import org.firstinspires.ftc.teamcode.Macros.Macro;
import org.firstinspires.ftc.teamcode.MotorPowers;
import org.firstinspires.ftc.teamcode.Robot;
import com.qualcomm.hardware.lynx.LynxModule;
import org.firstinspires.ftc.teamcode.ButtonMaps.BasicBM;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import java.io.File;

@TeleOp(name = "TeleOpWorkingBot")
public class TeleOpWorkingBot extends OpMode{

    VoltageSensor controlHubVoltageSensor;
    public Robot robot;
    WorkingBotButtonMap bm;
    private ElapsedTime lastFrameTime = new ElapsedTime();

    public double averageFrameTime = 0;
    private long frameCount;
    Limelight ll;
    GoToShoot goToShoot;

    MotorPowers mp;
    double[] vals;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, 10);
        bm = new WorkingBotButtonMap(this, robot);
        goToShoot = new GoToShoot(this);
        controlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
    }

    @Override
    public void loop() {
        for (LynxModule hub : robot.hubs) {
            hub.clearBulkCache();
        }
        telemetry.addData("macro active", goToShoot.active);
        if (!goToShoot.active) {
            mp = bm.getPowers();
            robot.setPowers(mp);
            vals = bm.getState();
            robot.setShootRPM(vals[0]);
            robot.setIntakePower(vals[1]);
            robot.setShootFeedPower(vals[2]);
            robot.setFlowerState(vals[3]);
            if (robot.macroActive) goToShoot.activate();
        } else {
            mp = goToShoot.nextFrame();
            vals = new double[4];
        }

        averageFrameTime = averageFrameTime * frameCount + lastFrameTime.milliseconds();
        averageFrameTime /= frameCount + 1;
        lastFrameTime.reset();
        frameCount ++;
        if (frameCount % 30 == 0){

            StringBuilder logData = new StringBuilder();
            logData.append("Time (s),Voltage (V)\n");
            double currentVoltage = controlHubVoltageSensor.getVoltage();
            logData.append(",").append(currentVoltage).append("\n");

            // Also show it on the Driver Station
            telemetry.addData("Voltage", "%.2f V", currentVoltage);
            telemetry.update();

            File logFile = AppUtil.getInstance().getSettingsFile("battery_log.csv");
            ReadWriteFile.writeFile(logFile, logData.toString());
        }
        TeleOpData.frameTime = averageFrameTime;
        telemetry.addData("Motors", "shoot motor target(%.2f), actual(%.2f)",vals[0], robot.shootMotor.getVelocity() * 60 /1075.2 );
        telemetry.addData("Motors", "left front(%.2f), right front(%.2f)", mp.lf, mp.rf);
        telemetry.addData("Motors", "left back(%.2f), right back(%.2f)", mp.lb, mp.rb);
        telemetry.addData("Motors", "enabled(%b)", robot.inputHandlingEnabled);
    }
}
