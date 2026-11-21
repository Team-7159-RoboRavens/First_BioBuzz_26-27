package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// All other poses are in ThreeCycleBlueBase
@Autonomous(name = "ThreeCycleBlueRightPark")
public class ThreeCycleBlueRightPark extends ThreeCycleBlueBase {
    private final Pose parkPose = new Pose(131.5075414781297, 45.2051282051282, Math.toRadians(180));

    @Override protected Pose getParkPose() { return parkPose; }
}
