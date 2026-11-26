package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.math.Pose;

// Starts on the other side of our HIVE from the three cycle, where our CELL starts down. Waits for the HIVE to tip,
// shoots, picks up the flower next to it and shoots at shoot pose 2 like three cycle volley 3, then parks
public abstract class BackSideBlueBase extends ThreeCycleBlueBase {
    // Three cycle start flipped across the field center line, 6 in from the field center along x
    protected static final Pose BACK_START_POSE = new Pose(72 + 6, 144 - SHOOT_POSE_1.y(), Math.toRadians(-90));
    // 3 in toward the HIVE like the three cycle MOVE, turned so the shooter faces our CELL (teleop aim math)
    protected static final Pose BACK_SHOT_POSE = aimedPose(BACK_START_POSE.x(), BACK_START_POSE.y() + 3, false);
    protected static final double BACK_SHOT_RPM = aimedRPM(BACK_SHOT_POSE, false);

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
