package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// Picks up the extra flower on the park wall after shoot pose 2, then parks right, see BackSideBlueBase
@Autonomous(name = "BackSideBlueRightPark")
public class BackSideBlueRightPark extends BackSideBlueBase {
    private final Pose parkPose = new Pose(131.5075414781297, 45.2051282051282, Math.toRadians(180));

    @Override protected Pose getParkPose() { return parkPose; }

    @Override
    protected PathState afterThirdVolley() {
        return PathState.EXTRA_APPROACH;
    }
}
