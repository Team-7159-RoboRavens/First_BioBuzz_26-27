package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// All other poses are in ThreeCycleBlueBase
@Autonomous(name = "ThreeCycleBlueLeftPark")
public class ThreeCycleBlueLeftPark extends ThreeCycleBlueBase {
    private final Pose parkPose = new Pose(130.9674, 20.8556, Math.toRadians(180));

    // 10 in closer to shoot pose 2 than the shared park control
    private final Pose parkControl = new Pose(103.6029, 44.2966);

    @Override protected Pose getParkPose() { return parkPose; }
    @Override protected Pose getParkControl() { return parkControl; }
}
