package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// Parks left from shoot pose 2 without the extra flower, see BackSideBlueBase
@Autonomous(name = "BackSideBlueLeftPark")
public class BackSideBlueLeftPark extends BackSideBlueBase {
    // Same as ThreeCycleBlueLeftPark
    private final Pose parkPose = new Pose(130.9674, 20.8556, Math.toRadians(180));
    private final Pose parkControl = new Pose(103.6029, 44.2966);

    @Override protected Pose getParkPose() { return parkPose; }
    @Override protected Pose getParkControl() { return parkControl; }
}
