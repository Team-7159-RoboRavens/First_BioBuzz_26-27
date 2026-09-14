package org.firstinspires.ftc.teamcode.Helpers;

import org.firstinspires.ftc.teamcode.Robot;

public class HeadingHelper {
    private Robot robot;

    public double heading;
    public HeadingHelper(Robot r) {
        robot = r;
        heading = (robot.pinpointAngle() + robot.limelightAngle())/2;
    }

    public void update() {
        heading = (robot.pinpointAngle() + robot.limelightAngle())/2;
    }
}
