package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// Three cycle plus an extra pickup from the park wall flower, shared poses are in ThreeCycleBlueBase
@Autonomous(name = "ExtraLoadAutoBlue")
public class ExtraLoadAutoBlue extends ExtraLoadAutoBase {
    private final Pose parkPose = new Pose(131.5075414781297, 45.2051282051282, Math.toRadians(180));
    private final Pose extraParkControl = new Pose(120, 68);

    @Override protected Pose getShootPose1() { return ThreeCycleBlueBase.SHOOT_POSE_1; }
    @Override protected Pose getShootPose1_0() { return ThreeCycleBlueBase.SHOOT_POSE_1_0; }
    @Override protected Pose getShootPose1_1() { return ThreeCycleBlueBase.SHOOT_POSE_1_1; }
    @Override protected Pose getIntakePose() { return ThreeCycleBlueBase.INTAKE_POSE; }
    @Override protected Pose getIntakeControl() { return ThreeCycleBlueBase.INTAKE_CONTROL; }
    @Override protected Pose getPreShootPose2() { return ThreeCycleBlueBase.PRE_SHOOT_POSE_2; }
    @Override protected Pose getShootPose2() { return ThreeCycleBlueBase.SHOOT_POSE_2; }
    @Override protected Pose getShoot2Control() { return ThreeCycleBlueBase.SHOOT_2_CONTROL; }
    @Override protected Pose getShoot2Control2() { return ThreeCycleBlueBase.SHOOT_2_CONTROL_2; }
    @Override protected Pose getParkControl() { return ThreeCycleBlueBase.PARK_CONTROL; }
    @Override protected Pose getParkPose() { return parkPose; }
    // Fixed here so the 3.5 in shoot 2 shift doesn't push the flower pickup into the wall
    private final Pose extraApproachPose = new Pose(129.20221530920061, 91.19909502262443, Math.toRadians(10));
    private final Pose extraIntakeEndPose = new Pose(131.6, 93, Math.toRadians(10));

    @Override protected Pose getExtraApproachPose() { return extraApproachPose; }
    @Override protected Pose getExtraIntakeEndPose() { return extraIntakeEndPose; }
    @Override protected Pose getExtraParkControl() { return extraParkControl; }
}
