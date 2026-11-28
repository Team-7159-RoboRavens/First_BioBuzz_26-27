package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

// Spins only the "shooter" motor, no Robot class
// g1: dpad up +1000, dpad right +100, Y +10, B +5, A +1, dpad down = 0
@TeleOp(name = "ShooterRpmTest", group = "Tuning")
public class ShooterRpmTest extends LinearOpMode {
    // 28 for a bare goBILDA/REV motor, change if geared
    public static double TICKS_PER_REV = 28.0;

    @Override
    public void runOpMode() {
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        double targetRpm = 0;
        boolean lastUp = false, lastRight = false, lastDown = false;
        boolean lastY = false, lastB = false, lastA = false;

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Add on press, not hold
            if (gamepad1.dpad_up && !lastUp) targetRpm += 1000;
            if (gamepad1.dpad_right && !lastRight) targetRpm += 100;
            if (gamepad1.y && !lastY) targetRpm += 10;
            if (gamepad1.b && !lastB) targetRpm += 5;
            if (gamepad1.a && !lastA) targetRpm += 1;
            if (gamepad1.dpad_down && !lastDown) targetRpm = 0;

            lastUp = gamepad1.dpad_up;
            lastRight = gamepad1.dpad_right;
            lastY = gamepad1.y;
            lastB = gamepad1.b;
            lastA = gamepad1.a;
            lastDown = gamepad1.dpad_down;

            if (targetRpm == 0) {   
                shooter.setPower(0);
            } else {
                shooter.setVelocity(targetRpm / 60.0 * TICKS_PER_REV);
            }

            double actualRpm = shooter.getVelocity() / TICKS_PER_REV * 60.0;
            telemetry.addData("Target RPM", "%.0f", targetRpm);
            telemetry.addData("Actual RPM", "%.0f", actualRpm);
            telemetry.addData("Power", "%.2f", shooter.getPower());
            telemetry.update();
        }

        shooter.setPower(0);
    }
}
