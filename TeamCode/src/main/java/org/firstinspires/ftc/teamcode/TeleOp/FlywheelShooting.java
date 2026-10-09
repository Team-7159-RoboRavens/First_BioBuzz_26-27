package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name="FlywheelShooting")
public class FlywheelShooting extends OpMode {
    Robot robot;

    // all units are m or radians
    double shotAngle = Math.toRadians(70);
    double launchHeight = 0.3;
    double hiveHeight = 1.2;
    double dist = 1.25;
    static double shootConstant = 279;
    static double servoConstant = 135.7;
    static double servoPower = 0.6;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, 0);
    }

    @Override
    public void loop() {
        double v = velocityShot(dist, hiveHeight - launchHeight, shotAngle);
        if (gamepad1.x) {
            robot.flywheelMotor.setVelocity(v);
        } else {
            robot.flywheelMotor.setVelocity(0);
        }
        if (gamepad1.b) {
            robot.shootFeedServo.setPower(servoPower);
        } else {
            robot.shootFeedServo.setPower(0);
        }
        if (gamepad1.dpad_up) {
            hiveHeight += 0.002;
        }
        if (gamepad1.dpad_down) {
            hiveHeight -= 0.002;
        }
        if (gamepad1.dpad_left) {
            dist -= 0.002;
        }
        if (gamepad1.dpad_right) {
            dist += 0.002;
        }
        if (gamepad1.left_bumper) {
            shootConstant -= 1;
        }
        if (gamepad1.right_bumper) {
            shootConstant += 1;
        }
        if (gamepad1.y) {
            servoPower += 0.01;
        }
        if (gamepad1.a) {
            servoPower -= 0.01;
        }
        if (Math.abs(gamepad1.left_stick_x) > 0.1) {
            servoConstant += gamepad1.left_stick_x/10;
        }
        if (Math.abs(gamepad1.left_stick_y) > 0.1) {
            shotAngle += gamepad1.left_stick_y/100;
        }
        telemetry.addLine("Press x to shoot, b to use servo");
        telemetry.addData("Hive height (dpad up/down)", hiveHeight);
        telemetry.addData("Distance to hive (dpad left/right)", dist);
        telemetry.addData("Shoot constant (L/R bumper)", shootConstant);
        telemetry.addData("Servo power (y/a)", servoPower);
        telemetry.addData("Servo constant (left joystick x)", servoConstant);
        telemetry.addData("Exit angle (left joystick y)", Math.toDegrees(shotAngle));
        telemetry.addData("Exit height", launchHeight);
        telemetry.addData("Target Velocity", v);
        telemetry.addData("Actual velocity", robot.flywheelMotor.getVelocity());
    }

    /**
     * @param dist Horizontal distance from launcher to target (m)
     * @param height Vertical distance from launcher to target (m)
     * @param pitch angle above the ground of the launcher (rad)
     */
    public static double velocityShot(double dist, double height, double pitch) {
        return shootConstant * dist * Math.sqrt(9.8/(2*(dist*Math.tan(pitch)-height))) / Math.cos(pitch) - servoConstant * servoPower;
    }
}
