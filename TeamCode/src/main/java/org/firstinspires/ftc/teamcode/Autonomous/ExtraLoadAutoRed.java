package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// Three cycle plus an extra pickup from the park wall flower, shared poses are in ThreeCycleRedBase
@Autonomous(name = "ExtraLoadAutoRed")
public class ExtraLoadAutoRed extends ExtraLoadAutoBase {
    private final Pose parkPose = new Pose(12.4924585218703, 98.7948717948718, Math.toRadians(0));
    private final Pose extraParkControl = new Pose(24, 76);

    @Override protected Pose getShootPose1() { return ThreeCycleRedBase.SHOOT_POSE_1; }
    @Override protected Pose getShootPose1_0() { return ThreeCycleRedBase.SHOOT_POSE_1_0; }
    @Override protected Pose getShootPose1_1() { return ThreeCycleRedBase.SHOOT_POSE_1_1; }
    @Override protected Pose getIntakePose() { return ThreeCycleRedBase.INTAKE_POSE; }
    @Override protected Pose getIntakeControl() { return ThreeCycleRedBase.INTAKE_CONTROL; }
    @Override protected Pose getPreShootPose2() { return ThreeCycleRedBase.PRE_SHOOT_POSE_2; }
    @Override protected Pose getShootPose2() { return ThreeCycleRedBase.SHOOT_POSE_2; }
    @Override protected Pose getShoot2Control() { return ThreeCycleRedBase.SHOOT_2_CONTROL; }
    @Override protected Pose getShoot2Control2() { return ThreeCycleRedBase.SHOOT_2_CONTROL_2; }
    @Override protected Pose getParkControl() { return ThreeCycleRedBase.PARK_CONTROL; }
    @Override protected Pose getParkPose() { return parkPose; }
    // Fixed here so the 3.5 in shoot 2 shift doesn't push the flower pickup into the wall
    private final Pose extraApproachPose = new Pose(16.29778469079939, 52.80090497737557, Math.toRadians(190));
    private final Pose extraIntakeEndPose = new Pose(12.5, 51.80090497737557, Math.toRadians(190));

    @Override protected Pose getExtraApproachPose() { return extraApproachPose; }
    @Override protected Pose getExtraIntakeEndPose() { return extraIntakeEndPose; }
    @Override protected Pose getExtraParkControl() { return extraParkControl; }
}
