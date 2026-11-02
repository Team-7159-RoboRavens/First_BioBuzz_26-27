package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.Limelight.calculatePosition.toPedroPose;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MotorPowers;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Limelight.calculatePosition;




public class SlowingDownonBoundary extends Robot{
    private Gamepad g1;
    public SlowingDownonBoundary(HardwareMap hmap, int robotId) {
        super(hmap, robotId);
    }

    public void loop() {
        //stuff you already wrote remember to import mp + robot
        Double pedroPose[3] = toPedroPose();
        double x = pedroPose[1];
        double y = pedroPose[2];
        double rate;

        if(x < 2) {
            rate = x*0.5;
        setPowers();
        if (g1.left_stick_x > 0); {
            setPowers();
            }
        }
        if(y < 2) {
            rate = y*0.5;
            setPowers();
            if (g1.left_stick_y > 0) {
                setPowers();
            }
        }
        if(x > 139.5)
            rate = (141.5-x)*0.5;
            setPowers();
        if (g1.left_stick_y < 0)
            setPowers();
        if(y > 139.5) {
            rate = (141.5 - y) * 0.5;
            setPowers();
            if (g1.left_stick_x > 0)
                setPowers();
        }
    }
}
