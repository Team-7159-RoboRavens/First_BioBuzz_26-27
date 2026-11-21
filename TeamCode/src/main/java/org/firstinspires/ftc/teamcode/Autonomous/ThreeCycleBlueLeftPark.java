package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// All other poses are in ThreeCycleBlueBase
@Autonomous(name = "ThreeCycleBlueLeftPark")
public class ThreeCycleBlueLeftPark extends ThreeCycleBlueBase {
    private final Pose parkPose = new Pose(131.5075414781297, 24.5, Math.toRadians(180));

    // 10 in closer to shoot pose 2 than the shared park control
    private final Pose parkControl = new Pose(104.143, 48.941);

    @Override protected Pose getParkPose() { return parkPose; }
    @Override protected Pose getParkControl() { return parkControl; }
}
