package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// ThreeCycleAutoBlue plus an extra pickup from the park wall flower (first 10 poses match ThreeCycleAutoBlue)
@Autonomous(name = "ExtraLoadAutoBlue")
public class ExtraLoadAutoBlue extends ExtraLoadAutoBase {
    private final Pose shootPose1 = new Pose(89.3, 127, Math.toRadians(90));
    private final Pose shootPose1_1 = new Pose(85.3, 117, Math.toRadians(0));

    private final Pose intakePose = new Pose(130.08898944193066, 128.5, Math.toRadians(0));
    private final Pose intakeControl = new Pose(96.3, 134.29);
    // -76 / -90 instead of 284 / 270 so the turn is the short way
    private final Pose preShootPose2 = new Pose(91.19909502262443, 15.49778469079939, Math.toRadians(-76));
    private final Pose shootPose2 = new Pose(92.19909502262443, 11.8, Math.toRadians(-76));
    private final Pose shoot2Control = new Pose(115.1184012066365, 44.029411764705884);
    private final Pose shoot2Control2 = new Pose(115.1184012066365, 44.029411764705884);
    private final Pose parkPose = new Pose(131.5075414781297, 45.2051282051282, Math.toRadians(180));
    private final Pose parkControl = new Pose(107.08220211161387, 58.49924585218703);

    // preShootPose2 / shootPose2 rotated 90 deg about (72, 72): x = 144 - y, y = x, heading + 90
    private final Pose extraApproachPose = new Pose(128.50221530920061, 91.19909502262443, Math.toRadians(14));
    private final Pose extraIntakeEndPose = new Pose(132, 92.19909502262443, Math.toRadians(14));
    private final Pose extraParkControl = new Pose(120, 68);

    @Override protected Pose getShootPose1() { return shootPose1; }
    @Override protected Pose getShootPose1_1() { return shootPose1_1; }
    @Override protected Pose getIntakePose() { return intakePose; }
    @Override protected Pose getIntakeControl() { return intakeControl; }
    @Override protected Pose getPreShootPose2() { return preShootPose2; }
    @Override protected Pose getShootPose2() { return shootPose2; }
    @Override protected Pose getShoot2Control() { return shoot2Control; }
    @Override protected Pose getShoot2Control2() { return shoot2Control2; }
    @Override protected Pose getParkPose() { return parkPose; }
    @Override protected Pose getParkControl() { return parkControl; }

    @Override protected Pose getExtraApproachPose() { return extraApproachPose; }
    @Override protected Pose getExtraIntakeEndPose() { return extraIntakeEndPose; }
    @Override protected Pose getExtraParkControl() { return extraParkControl; }
}
