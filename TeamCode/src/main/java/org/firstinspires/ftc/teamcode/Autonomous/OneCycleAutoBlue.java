package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "OneCycleAutoBlue")
public class OneCycleAutoBlue extends OneCycleAutoBase {
    @Override
    protected Pose getStartPose() {
        return new Pose(133, 117, Math.toRadians(180));
    }

    @Override
    protected Pose getShootPose1() {
        return new Pose(85.23906485671196, 117, Math.toRadians(270));
    }

    @Override
    protected Pose getIntakePose() {
        return new Pose(133.08898944193066, 133.62971342383105, Math.toRadians(0));
    }

    @Override
    protected Pose getIntakeControl() {
        return new Pose(64.13273001508298, 126.42458521870286);
    }

    @Override
    protected Pose getParkPose() {
        return new Pose(132.57466063348417, 43.497737556561084, Math.toRadians(0));
    }

    @Override
    protected Pose getParkControl() {
        return new Pose(109.37971342383108, 52.54227941176471);
    }
}
