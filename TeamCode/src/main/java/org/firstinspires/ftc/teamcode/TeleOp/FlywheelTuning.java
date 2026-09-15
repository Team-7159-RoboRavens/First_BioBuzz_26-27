package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.ButtonMaps.BasicBM;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name="FlywheelTuning")
public class FlywheelTuning extends OpMode {
    Robot robot;
    BasicBM bm;

    public double highVelocity = 1500;
    public double lowVelocity = 900;
    public double curTargetVelocity = highVelocity;
    public double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};
    int stepIndex = 1;
    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        bm = new BasicBM(this, robot);
        telemetry.addLine("Init complete");
    }

    @Override
    public void loop() {
        // get all our gamepad commands
        // set target velocity
        // update telemetry
        if (gamepad1.yWasPressed()){
            if (curTargetVelocity == highVelocity){
                curTargetVelocity = lowVelocity;
            } else { curTargetVelocity = highVelocity; }
        }

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()){
            robot.F -= stepSizes[stepIndex];
        }

        if (gamepad1.dpadRightWasPressed()){
            robot.F +=stepSizes[stepIndex];
        }

        if (gamepad1.dpadDownWasPressed()){
            robot.P += stepSizes[stepIndex];
        }

        if (gamepad1.dpadUpWasPressed()){
            robot.P -= stepSizes[stepIndex];
        }
        // set new PIDF coefficients
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(robot.P,0,0,robot.F);
        robot.flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        // set velocity
        robot.flywheelMotor.setVelocity(curTargetVelocity);

        double curVelocity = robot.flywheelMotor.getVelocity();
        double error = curTargetVelocity - curVelocity;

        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", error);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addLine("-----------------------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", robot.P);
        telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", robot.F);
        telemetry.addData("Step Size", ".%4f (B Button)", stepSizes[stepIndex]);

    }
}
