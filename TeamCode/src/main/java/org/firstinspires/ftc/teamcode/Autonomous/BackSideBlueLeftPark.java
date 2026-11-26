package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// Parks left from shoot pose 2 without the extra flower, see BackSideBlueBase
@Autonomous(name = "BackSideBlueLeftPark")
public class BackSideBlueLeftPark extends BackSideBlueBase {
    // Same as ThreeCycleBlueLeftPark
    private final Pose parkPose = new Pose(131.5075414781297, 15.5, Math.toRadians(180));
    private final Pose parkControl = new Pose(104.143, 38.941);

    @Override protected Pose getParkPose() { return parkPose; }
    @Override protected Pose getParkControl() { return parkControl; }
}
