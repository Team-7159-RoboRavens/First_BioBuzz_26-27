package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.math.Pose;

// Shared poses for every Red auto (Left Park, Right Park, Safe), only the park pose is set per auto
// Blue rotated 180 deg about (72, 72): x = 144 - x, y = 144 - y, heading + 180
public abstract class ThreeCycleRedBase extends ThreeCycleAutoBase {
    protected static final Pose SHOOT_POSE_1 = new Pose(54.7, 17, Math.toRadians(270));
    protected static final Pose SHOOT_POSE_1_1 = new Pose(58.7, 27, Math.toRadians(180));

    protected static final Pose INTAKE_POSE = new Pose(11.91101055806934, 15.5, Math.toRadians(180));
    protected static final Pose INTAKE_CONTROL = new Pose(47.7, 9.71);
    protected static final Pose PRE_SHOOT_POSE_2 = new Pose(52.80090497737557, 128.50221530920061, Math.toRadians(100));
    protected static final Pose SHOOT_POSE_2 = new Pose(51.80090497737557, 131.2, Math.toRadians(100));
    protected static final Pose SHOOT_2_CONTROL = new Pose(28.8815987933635, 99.970588235294116);
    protected static final Pose SHOOT_2_CONTROL_2 = new Pose(28.8815987933635, 99.970588235294116);
    protected static final Pose PARK_CONTROL = new Pose(36.91779788838613, 85.50075414781297);

    @Override protected Pose getShootPose1() { return SHOOT_POSE_1; }
    @Override protected Pose getShootPose1_1() { return SHOOT_POSE_1_1; }
    @Override protected Pose getIntakePose() { return INTAKE_POSE; }
    @Override protected Pose getIntakeControl() { return INTAKE_CONTROL; }
    @Override protected Pose getPreShootPose2() { return PRE_SHOOT_POSE_2; }
    @Override protected Pose getShootPose2() { return SHOOT_POSE_2; }
    @Override protected Pose getShoot2Control() { return SHOOT_2_CONTROL; }
    @Override protected Pose getShoot2Control2() { return SHOOT_2_CONTROL_2; }
    @Override protected Pose getParkControl() { return PARK_CONTROL; }
}
