package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ButtonMaps.PinpointBM;
import org.firstinspires.ftc.teamcode.Helpers.HeadingHelper;
import org.firstinspires.ftc.teamcode.Limelight.Limelight;
import org.firstinspires.ftc.teamcode.MotorPowers;
import org.firstinspires.ftc.teamcode.Robot;
import com.qualcomm.hardware.lynx.LynxModule;

// TeleOp op mode using the Pinpoint-aware button map, with odometry telemetry.
@TeleOp(name="TeleOpBaseP")
public class TeleOpBaseP extends OpMode {
    Robot robot;
    PinpointBM pm;
    HeadingHelper h;
    Limelight ll;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, 3);
        ll = new Limelight(robot);
        ll.switchPipeline(1);
        pm = new PinpointBM(this, robot);
        h = new HeadingHelper(robot);
    }

    @Override
    public void loop() {
        for (LynxModule hub : robot.hubs) {
            hub.clearBulkCache();
        }
        MotorPowers mp = pm.getPowers();
        robot.setPowers(mp);
        robot.setIntakePower(pm.getIntakePower());
        ll.update();


        telemetry.addData("Motors", "left front(%.2f), right front(%.2f)", mp.lf, mp.rf);
        telemetry.addData("Motors", "left back(%.2f), right back(%.2f)", mp.lb, mp.rb);
        telemetry.addData("Pinpoint Angle", robot.pinpointAngle());
        telemetry.addData("Pinpoint X", robot.getPinpointX());
        telemetry.addData("Pinpoint Y", robot.getPinpointY());
        telemetry.addData("Avg heading:", h.heading);
    }

}
