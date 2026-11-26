package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// Parks left from shoot pose 2 without the extra flower, see BackSideRedBase
@Autonomous(name = "BackSideRedLeftPark")
public class BackSideRedLeftPark extends BackSideRedBase {
    // Same as ThreeCycleRedLeftPark
    private final Pose parkPose = new Pose(12.4924585218703, 126.67, Math.toRadians(0));
    private final Pose parkControl = new Pose(40.014, 105.009);

    @Override protected Pose getParkPose() { return parkPose; }
    @Override protected Pose getParkControl() { return parkControl; }
}
