package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// All other poses are in ThreeCycleRedBase
@Autonomous(name = "ThreeCycleRedLeftPark")
public class ThreeCycleRedLeftPark extends ThreeCycleRedBase {
    private final Pose parkPose = new Pose(13.0326, 121.3144, Math.toRadians(0));

    // 10 in closer to shoot pose 2 than the shared park control
    private final Pose parkControl = new Pose(40.5541, 99.6534);

    @Override protected Pose getParkPose() { return parkPose; }
    @Override protected Pose getParkControl() { return parkControl; }
}
