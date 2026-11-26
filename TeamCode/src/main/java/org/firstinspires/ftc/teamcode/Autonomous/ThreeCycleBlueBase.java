package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.math.Pose;

// Shared poses for every Blue auto (three cycle, ExtraLoad, Safe, BackSide, GardenCycle), only the park pose is set per auto
public abstract class ThreeCycleBlueBase extends ThreeCycleAutoBase {
    protected static final Pose SHOOT_POSE_1 = new Pose(89.3, 127, Math.toRadians(90));
    protected static final Pose SHOOT_POSE_1_0 = new Pose(89.3, 124.0, Math.toRadians(90));
    protected static final Pose SHOOT_POSE_1_1 = new Pose(85.3, 117, Math.toRadians(0));

    protected static final Pose INTAKE_POSE = new Pose(132.58898944193066, 128.5, Math.toRadians(0));
    protected static final Pose INTAKE_CONTROL = new Pose(96.3, 134.29);
    // -80 / -90 instead of 280 / 270 so the turn is the short way
    // Pre-shoot 2 and shoot 2 moved 3.5 in toward the wall to match where the robot actually ends up, then pre-shoot 2 back out 1.5 in
    protected static final Pose PRE_SHOOT_POSE_2 = new Pose(91.19909502262443, 12.79778469079939, Math.toRadians(-80));
    // RPM for volleys 2 and 3 from the teleop RPM math (Shooter/ShotCalculator) at these two poses,
    // 41.0 in and 45.2 in from the opening. Redo them with that math if the poses move
    protected static final double SHOT_2_RPM = 197;
    protected static final double SHOT_3_RPM = 201;
    protected static final Pose SHOOT_POSE_2 = new Pose(93, 8.9, Math.toRadians(-80));
    protected static final Pose SHOOT_2_CONTROL = new Pose(115.1184012066365, 44.029411764705884);
    protected static final Pose SHOOT_2_CONTROL_2 = new Pose(115.1184012066365, 44.029411764705884);
    protected static final Pose PARK_CONTROL = new Pose(107.08220211161387, 58.49924585218703);

    // Extra flower on the park wall, set by hand so the 3.5 in shoot 2 shift doesn't push the pickup into the wall
    protected static final Pose EXTRA_APPROACH_POSE = new Pose(129.20221530920061, 91.19909502262443, Math.toRadians(10));
    protected static final Pose EXTRA_INTAKE_END_POSE = new Pose(131.6, 93, Math.toRadians(10));
    protected static final Pose EXTRA_PARK_CONTROL = new Pose(120, 68);

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
