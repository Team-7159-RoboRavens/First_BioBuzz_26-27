package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "OneCycleAutoRed")
public class OneCycleAutoRed extends OneCycleAutoBase {
    @Override
    protected Pose getStartPose() {
        return new Pose(9, 25, Math.toRadians(0));
    }

    @Override
    protected Pose getShootPose1() {
        return new Pose(59, 25, Math.toRadians(90));
    }

    @Override
    protected Pose getIntakePose() {
        return new Pose(8.876319758672755, 8.349924585218675, Math.toRadians(0));
    }

    @Override
    protected Pose getIntakeControl() {
        return new Pose(98.49396681749627, 13.523378582202092);
    }

    @Override
    protected Pose getParkPose() {
        return new Pose(10.923076923076934, 97.920814479638, Math.toRadians(280));
    }

    @Override
    protected Pose getParkControl() {
        return new Pose(24.0844645550528, 29.187782805429887);
    }
}
