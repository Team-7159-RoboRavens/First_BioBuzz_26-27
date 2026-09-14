package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ButtonMaps.BasicBM;
import org.firstinspires.ftc.teamcode.Limelight;
import org.firstinspires.ftc.teamcode.MotorPowers;
import org.firstinspires.ftc.teamcode.Robot;

// TeleOp op mode using the basic (non-Pinpoint) button map.
@TeleOp(name="TeleOpBase")
public class TeleOpBase extends OpMode {
    Robot robot;
    BasicBM bm;

    Limelight ll;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        ll = new Limelight(robot);
        bm = new BasicBM(this, robot);
    }

    @Override
    public void loop() {
        MotorPowers mp = bm.getPowers();
        robot.setPowers(mp);
        robot.setIntakePower(bm.getIntakePower());
        ll.update();
        if (ll.hasValidResult()) {
            int count = ll.getColorBlobCount();
            double y = ll.getColorBlobYOffset(0);
            double x = ll.getColorBlobXOffset(0);
            double xSize = ll.getColorBlobXSize(0);
            double ySize = ll.getColorBlobYSize(0);
            telemetry.addData("Limelight Targets", "Count: %d", count);
            telemetry.addData("Limelight Offset", "X: %.1f , Y: %.1f", x, y);
            telemetry.addData("Limelight Size", "W: %.1f , H: %.1f", xSize, ySize);
        }
        else {
            telemetry.addData("Limelight", "no result");
        }
        telemetry.addData("Motors", "left front(%.2f), right front(%.2f)", mp.lf, mp.rf);
        telemetry.addData("Motors", "left back(%.2f), right back(%.2f)", mp.lb, mp.rb);
    }
}
