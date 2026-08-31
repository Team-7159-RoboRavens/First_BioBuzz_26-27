package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ButtonMaps.BasicBM;
import org.firstinspires.ftc.teamcode.ButtonMaps.PinpointBM;
import org.firstinspires.ftc.teamcode.Limelight;
import org.firstinspires.ftc.teamcode.MotorPowers;
import org.firstinspires.ftc.teamcode.Robot;

// TeleOp op mode using the Pinpoint-aware button map, with odometry telemetry.
@TeleOp(name="TeleOpBaseP")
public class TeleOpBaseP extends OpMode {
    Robot robot;
    PinpointBM pm;
    Limelight ll;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        ll = new Limelight(robot);
        ll.switchPipeline(1);
        pm = new PinpointBM(this, robot);
    }

    @Override
    public void loop() {
        MotorPowers mp = pm.getPowers();
        robot.setPowers(mp);
        robot.setIntakePower(pm.getIntakePower());
        ll.update();
        if (ll.hasValidResult()) {
            int id = ll.getAprilTagId();
            double y = ll.getAprilTagAngleYOffset();
            double d2D = ll.getAprilTagDistance2D();
            double x = ll.getAprilTagAngleXOffset();
            double d3D = ll.getAprilTagDistance3D();
            telemetry.addData("Limelight ID", "ID: %.1f", id);
            telemetry.addData("Limelight Offset", "X: %.1f , Y: %.1f", x, y);
            telemetry.addData("Limelight Distance", "2D: %.1f , 3D: %.1f", d2D, d3D);
            telemetry.addData("Motors", "left front(%.2f), right front(%.2f)", mp.lf, mp.rf);
            telemetry.addData("Motors", "left back(%.2f), right back(%.2f)", mp.lb, mp.rb);
        }
        else {
            telemetry.addData("Limelight", "no result");
        }
        telemetry.addData("Pinpoint Angle", robot.pinpointAngle());
        telemetry.addData("Pinpoint X", robot.getPinpointX());
        telemetry.addData("Pinpoint Y", robot.getPinpointY());
    }
}
