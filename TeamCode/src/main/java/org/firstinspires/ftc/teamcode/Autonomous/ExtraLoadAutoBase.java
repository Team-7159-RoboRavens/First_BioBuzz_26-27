package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

// Three cycle auto plus an extra pickup from the flower on the park wall before parking
public abstract class ExtraLoadAutoBase extends ThreeCycleAutoBase {

    protected static final double EXTRA_APPROACH_TIMEOUT = 5.0;
    protected static final double EXTRA_INTAKE_TIMEOUT = 2.0;
    // Same as INTAKE2
    protected static final double EXTRA_PICKUP_TIME = 0.5;

    // Extra flower poses are preShootPose2 / shootPose2 rotated 90 deg about (72, 72): x = 144 - y, y = x, heading + 90
    protected static Pose rotate90(Pose p) {
        return new Pose(144 - p.y(), p.x(), p.heading() + Math.PI / 2);
    }

    protected Pose getExtraApproachPose() { return rotate90(getPreShootPose2()); }

    protected Pose getExtraIntakeEndPose() { return rotate90(getShootPose2()); }

    protected abstract Pose getExtraParkControl();

    @Override
    protected PathState afterThirdVolley() {
        return PathState.EXTRA_APPROACH;
    }

    // Park from the flower, curving away from the wall so the robot can turn
    @Override
    protected Path buildParkPath() {
        return curveTo(getExtraIntakeEndPose(), getExtraParkControl(), getParkPose());
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
