package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// Three cycle plus an extra pickup from the park wall flower before parking right, poses are in ThreeCycleRedBase
@Autonomous(name = "ExtraLoadAutoRed")
public class ExtraLoadAutoRed extends ThreeCycleRedBase {
    private final Pose parkPose = new Pose(13.0326, 93.4393, Math.toRadians(0));

    @Override protected Pose getParkPose() { return parkPose; }

    @Override
    protected PathState afterThirdVolley() {
        return PathState.EXTRA_APPROACH;
    }
}
