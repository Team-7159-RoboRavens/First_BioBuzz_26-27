package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// Picks up the extra flower on the park wall after shoot pose 2, then parks right, see BackSideRedBase
@Autonomous(name = "BackSideRedRightPark")
public class BackSideRedRightPark extends BackSideRedBase {
    private final Pose parkPose = new Pose(13.0326, 93.4393, Math.toRadians(0));

    @Override protected Pose getParkPose() { return parkPose; }

    @Override
    protected PathState afterThirdVolley() {
        return PathState.EXTRA_APPROACH;
    }
}
