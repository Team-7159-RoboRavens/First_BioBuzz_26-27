package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.math.Pose;

// BackSideBlueBase rotated 180 deg about (72, 72), see there
public abstract class BackSideRedBase extends ThreeCycleRedBase {
    protected static final Pose BACK_START_POSE = new Pose(
            144 - BackSideBlueBase.BACK_START_POSE.x(), 144 - BackSideBlueBase.BACK_START_POSE.y(), Math.toRadians(90));
    protected static final Pose BACK_SHOT_POSE = aimedPose(BACK_START_POSE.x(), BACK_START_POSE.y() - 3, true);
    protected static final double BACK_SHOT_RPM = aimedRPM(BACK_SHOT_POSE, true);

    @Override protected Pose getStartPose() { return BACK_START_POSE; }
    @Override protected Pose getFirstShotPose() { return BACK_SHOT_POSE; }
    @Override protected double getFirstShotRPM() { return BACK_SHOT_RPM; }
    @Override protected PathState getFirstState() { return PathState.WAIT_FOR_TIP; }
    @Override protected PathState afterFirstVolley() { return PathState.BACK_TO_PICKUP; }

    // Both volleys wait for our CELL on this side to be up
    @Override
    protected boolean waitForUpBefore(PathState shot) {
        return shot == PathState.SHOOT_PRELOAD1 || shot == PathState.SHOOT_PRELOAD3 || super.waitForUpBefore(shot);
    }
}
