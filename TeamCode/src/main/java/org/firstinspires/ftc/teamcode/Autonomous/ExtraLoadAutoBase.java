package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

// Three cycle auto plus an extra pickup from the flower on the park wall before parking
public abstract class ExtraLoadAutoBase extends ThreeCycleAutoBase {

    protected static final double EXTRA_APPROACH_TIMEOUT = 4.0;
    protected static final double EXTRA_INTAKE_TIMEOUT = 1.5;
    // Same as INTAKE2
    protected static final double EXTRA_PICKUP_TIME = 0.5;

    protected abstract Pose getExtraApproachPose();

    protected abstract Pose getExtraIntakeEndPose();

    @Override
    protected PathState afterThirdVolley() {
        return PathState.EXTRA_APPROACH;
    }

    // Park from the flower instead of from the shoot pose
    @Override
    protected Path buildParkPath() {
        return lineTo(getExtraIntakeEndPose(), getParkPose());
    }

    @Override
    protected void handleExtraState() {
        switch (pathState) {
            case EXTRA_APPROACH:
                mechanisms.intake();
                if (runPath(() -> lineTo(getShootPose2(), getExtraApproachPose()),
                        EXTRA_APPROACH_TIMEOUT)) {
                    mechanisms.lowerFlower();
                    setPathState(PathState.EXTRA_INTAKE);
                }
                break;

            case EXTRA_INTAKE:
                mechanisms.intake();
                if (runPath(() -> lineTo(getExtraApproachPose(), getExtraIntakeEndPose()),
                        EXTRA_INTAKE_TIMEOUT)) {
                    setPathState(PathState.EXTRA_PICKUP);
                }
                break;

            case EXTRA_PICKUP:
                mechanisms.intake();
                if (pathTimer.seconds() >= EXTRA_PICKUP_TIME) {
                    setPathState(PathState.SHOOT_POS2_PARK_POS);
                }
                break;

            default:
                break;
        }
    }
}
