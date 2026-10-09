package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Limelight.Limelight;
import org.firstinspires.ftc.teamcode.Limelight.LimelightBlue;

@TeleOp(name="LimelightShooting")
public class LimelightShooting extends FlywheelShooting {
    Limelight ll;
    LimelightBlue llb;

    @Override
    public void init() {
        ll = new Limelight(robot);
        ll.useAprilTagPipeline();
        llb = new LimelightBlue(ll, telemetry);
        ll.useAprilTagPipeline();
        ll.useColorPipeline();
        telemetry.setMsTransmissionInterval(11);
    }

    @Override
    public void loop() {
        dist = getDist(llb.aimState(llb.getPosition())); // TODO: finish ts
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
        if (gamepad1.y) {
            servoPower += 0.01;
        }
        if (gamepad1.a) {
            servoPower -= 0.01;
        }
        telemetry.addData("Hive height", hiveHeight);
        telemetry.addData("Distance to hive", dist);
        telemetry.addData("Servo power (y/a)", servoPower);
        telemetry.addData("Target Velocity", v);
        telemetry.addData("Actual velocity", robot.flywheelMotor.getVelocity());
    }

    public double getDist(double[] llDist) {
        return llDist[0] * llDist[0] + llDist[1] * llDist[1];
    }
}
