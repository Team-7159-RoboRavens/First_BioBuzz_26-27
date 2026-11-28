package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.math.Pose;

// Shared poses for every Blue auto (three cycle, ExtraLoad, Safe, BackSide, GardenCycle), only the park pose is set per auto
public abstract class ThreeCycleBlueBase extends ThreeCycleAutoBase {
    // Start: intake on the y = 144 wall, park side edge 46.770 in - 46.989 mm from the park wall
    protected static final Pose SHOOT_POSE_1 = new Pose(88.7599, 132.3556, Math.toRadians(90));
    protected static final Pose SHOOT_POSE_1_0 = new Pose(88.7599, 129.3556, Math.toRadians(90));
    protected static final Pose SHOOT_POSE_1_1 = new Pose(84.7599, 122.3556, Math.toRadians(0));

    protected static final Pose INTAKE_POSE = new Pose(132.0489, 133.8556, Math.toRadians(0));
    protected static final Pose INTAKE_CONTROL = new Pose(95.7599, 139.6456);
    // -80 / -90 instead of 280 / 270 so the turn is the short way
    // Pre-shoot 2 and shoot 2 moved 3.5 in toward the wall to match where the robot actually ends up, then pre-shoot 2 back out 1.5 in
    protected static final Pose PRE_SHOOT_POSE_2 = new Pose(90.659, 18.1534, Math.toRadians(-80));
    // RPM for volleys 2 and 3, from the teleop RPM math (Shooter/ShotCalculator) before the start pose fix.
    // These poses are 35.7 in and 39.9 in from the opening, where that math now gives about 192
    protected static final double SHOT_2_RPM = 197;
    protected static final double SHOT_3_RPM = 201;
    protected static final Pose SHOOT_POSE_2 = new Pose(92.4599, 14.2556, Math.toRadians(-80));
    protected static final Pose SHOOT_2_CONTROL = new Pose(114.5783, 49.385);
    protected static final Pose SHOOT_2_CONTROL_2 = new Pose(114.5783, 49.385);
    protected static final Pose PARK_CONTROL = new Pose(106.5421, 63.8548);

    // Extra flower on the park wall, set by hand so the 3.5 in shoot 2 shift doesn't push the pickup into the wall
    protected static final Pose EXTRA_APPROACH_POSE = new Pose(128.6621, 96.5547, Math.toRadians(10));
    protected static final Pose EXTRA_INTAKE_END_POSE = new Pose(131.0599, 98.3556, Math.toRadians(10));
    protected static final Pose EXTRA_PARK_CONTROL = new Pose(119.4599, 73.3556);

    @Override protected Pose getShootPose1() { return SHOOT_POSE_1; }
    @Override protected Pose getShootPose1_0() { return SHOOT_POSE_1_0; }
    @Override protected Pose getShootPose1_1() { return SHOOT_POSE_1_1; }
    @Override protected Pose getIntakePose() { return INTAKE_POSE; }
    @Override protected Pose getIntakeControl() { return INTAKE_CONTROL; }
    @Override protected Pose getPreShootPose2() { return PRE_SHOOT_POSE_2; }
    @Override protected Pose getShootPose2() { return SHOOT_POSE_2; }
    @Override protected Pose getShoot2Control() { return SHOOT_2_CONTROL; }
    @Override protected Pose getShoot2Control2() { return SHOOT_2_CONTROL_2; }
    @Override protected Pose getParkControl() { return PARK_CONTROL; }
    @Override protected double getShot2RPM() { return SHOT_2_RPM; }
    @Override protected double getShot3RPM() { return SHOT_3_RPM; }
    @Override protected Pose getExtraApproachPose() { return EXTRA_APPROACH_POSE; }
    @Override protected Pose getExtraIntakeEndPose() { return EXTRA_INTAKE_END_POSE; }
    @Override protected Pose getExtraParkControl() { return EXTRA_PARK_CONTROL; }
    @Override protected boolean isRed() { return false; }
}
