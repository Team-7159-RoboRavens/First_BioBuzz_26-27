package org.firstinspires.ftc.teamcode.Macros;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.ButtonMaps.Mechanumdrive;
import org.firstinspires.ftc.teamcode.MotorPowers;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOpWorkingBot;

public class GoToShoot extends Macro {
    private final Pose shootPose1;
    private final Pose shootPose2;
    private final TeleOpWorkingBot teleOp;
    private Robot robot;
    double xDiff;
    double yDiff;
    double dist;
    double dist0;
    double progress;
    double factor;
    double xPower;
    double yPower;
    public boolean active;
    public GoToShoot(TeleOpWorkingBot op) {
        active = false;
        teleOp = op;
        robot = teleOp.robot;
        shootPose1 = new Pose(89.3, 127, Math.toRadians(90));
        shootPose2 = new Pose(92.19909502262443, 12, Math.toRadians(284));
        xDiff = shootPose1.x() - robot.getPinpointX();
        yDiff = shootPose1.x() - robot.getPinpointY();
        dist = Math.sqrt(xDiff*xDiff + yDiff*yDiff);
        dist0 = dist;
        progress = (dist0-dist)/dist0;
        factor = Math.pow(Math.sin(Math.PI*progress),2)+0.1;
        xPower = xDiff/dist;
        yPower = yDiff/dist;
        robot.pinpointUpdate();
        robot.setPowers(Mechanumdrive.fieldOrientedDrive(yPower*factor, xPower*factor, 0, Math.PI/2));
    }

    public void updateProgress() {
        // TODO: add turning
        robot.pinpointUpdate();
        xDiff = shootPose1.x() - robot.getPinpointX();
        yDiff = shootPose1.x() - robot.getPinpointY();
        dist = Math.sqrt(xDiff*xDiff + yDiff*yDiff);
        progress = (dist0-dist)/dist0;
        factor = Math.pow(Math.sin(Math.PI*progress),2);
        xPower = xDiff/dist;
        yPower = yDiff/dist;
        if (progress > 0.9) {
            active = false;
            xPower = 0;
            yPower = 0;
        }
    }

    public MotorPowers nextFrame() {
        updateProgress();
        return Mechanumdrive.fieldOrientedDrive(yPower*factor, xPower*factor, 0, Math.PI/2);
    }

    public void activate() {
        active = true;
    }
}
