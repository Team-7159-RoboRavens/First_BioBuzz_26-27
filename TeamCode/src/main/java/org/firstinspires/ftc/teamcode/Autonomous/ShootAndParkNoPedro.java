package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ButtonMaps.Mechanumdrive;
import org.firstinspires.ftc.teamcode.MotorPowers;
import org.firstinspires.ftc.teamcode.Robot;

// Timed auto that doesn't use Pedro Pathing: spin up the shooter, feed for a few seconds, then drive off the line to park.
@Autonomous(name = "ShootAndParkNoPedro")
public class ShootAndParkNoPedro extends LinearOpMode {
    private static final int ROBOT_ID = 10;

    private static final double SHOOT_RPM = 200;
    private static final long SPIN_UP_MS = 2000;

    private static final long FEED_MS = 5000;
    private static final double FEED_POWER = 0.7;
    private static final double INTAKE_POWER = 1;

    // Starting values for the park drive, tune these on the robot.
    // The angle is measured counterclockwise from straight right, so 70 degrees is mostly forward and a little left.
    private static final double PARK_ANGLE_DEGREES = 65;
    private static final double PARK_POWER = 0.7;
    private static final long PARK_MS = 2500;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, ROBOT_ID);
        telemetry.addLine("ready");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) {
            return;
        }

        telemetry.addLine("spinning up shooter");
        telemetry.update();
        robot.setShootRPM(SHOOT_RPM);
        sleep(SPIN_UP_MS);

        telemetry.addLine("feeding");
        telemetry.update();
        robot.setShootFeedPower(FEED_POWER);
        robot.setIntakePower(INTAKE_POWER);
        sleep(FEED_MS);

        robot.setShootFeedPower(0);
        robot.setIntakePower(0);
        robot.setShootRPM(0);

        telemetry.addLine("parking");
        telemetry.update();
        double parkAngle = Math.toRadians(PARK_ANGLE_DEGREES);
        double forward = -Math.sin(parkAngle) * PARK_POWER;
        double right = Math.cos(parkAngle) * PARK_POWER;
        robot.setPowers(Mechanumdrive.setMotorPowers(forward, right, 0));
        sleep(PARK_MS);
        robot.setPowers(new MotorPowers(0, 0, 0, 0));

        telemetry.addLine("done");
        telemetry.update();
    }
}
