package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.math.Pose;

// Shared poses for every Red auto (three cycle, ExtraLoad, Safe, BackSide, GardenCycle), only the park pose is set per auto
// Blue rotated 180 deg about (72, 72): x = 144 - x, y = 144 - y, heading + 180
public abstract class ThreeCycleRedBase extends ThreeCycleAutoBase {
    protected static final Pose SHOOT_POSE_1 = new Pose(55.2401, 11.6444, Math.toRadians(270));
    protected static final Pose SHOOT_POSE_1_0 = new Pose(55.2401, 14.6444, Math.toRadians(270));
    protected static final Pose SHOOT_POSE_1_1 = new Pose(59.2401, 21.6444, Math.toRadians(180));

    protected static final Pose INTAKE_POSE = new Pose(12.4511, 10.1444, Math.toRadians(180));
    protected static final Pose INTAKE_CONTROL = new Pose(48.2401, 4.3544);
    // Pre-shoot 2 and shoot 2 moved 3.5 in toward the wall to match where the robot actually ends up, then pre-shoot 2 back out 1.5 in
    protected static final Pose PRE_SHOOT_POSE_2 = new Pose(53.341, 124.3466, Math.toRadians(100));
    // RPM for volleys 2 and 3, from the teleop RPM math (Shooter/ShotCalculator) before the start pose fix.
    // These poses are 34.2 in and 39.6 in from the opening, where that math now gives about 192
    protected static final double SHOT_2_RPM = 195;
    protected static final double SHOT_3_RPM = 200;
    protected static final Pose SHOOT_POSE_2 = new Pose(52.341, 129.6444, Math.toRadians(100));
    protected static final Pose SHOOT_2_CONTROL = new Pose(29.4217, 94.615);
    protected static final Pose SHOOT_2_CONTROL_2 = new Pose(29.4217, 94.615);
    protected static final Pose PARK_CONTROL = new Pose(37.4579, 80.1452);

    // Extra flower on the park wall, set by hand so the 3.5 in shoot 2 shift doesn't push the pickup into the wall
    protected static final Pose EXTRA_APPROACH_POSE = new Pose(16.8379, 47.4453, Math.toRadians(190));
    protected static final Pose EXTRA_INTAKE_END_POSE = new Pose(13.0401, 46.4453, Math.toRadians(190));
    protected static final Pose EXTRA_PARK_CONTROL = new Pose(24.5401, 70.6444);

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
    @Override protected boolean isRed() { return true; }
}
