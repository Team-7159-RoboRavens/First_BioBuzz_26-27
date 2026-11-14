package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// Blue rotated 180 deg about (72, 72): x = 144 - x, y = 144 - y, heading + 180
@Autonomous(name = "ExtraLoadAutoRed")
public class ExtraLoadAutoRed extends ExtraLoadAutoBase {
    private final Pose shootPose1 = new Pose(54.7, 17, Math.toRadians(270));
    private final Pose shootPose1_1 = new Pose(58.7, 27, Math.toRadians(180));

    private final Pose intakePose = new Pose(18.91101055806934, 15.5, Math.toRadians(180));
    private final Pose intakeControl = new Pose(47.7, 9.71);
    private final Pose preShootPose2 = new Pose(52.80090497737557, 128.50221530920061, Math.toRadians(104));
    private final Pose shootPose2 = new Pose(51.80090497737557, 132, Math.toRadians(104));
    private final Pose shoot2Control = new Pose(28.8815987933635, 99.970588235294116);
    private final Pose shoot2Control2 = new Pose(28.8815987933635, 99.970588235294116);
    private final Pose parkPose = new Pose(12.4924585218703, 98.7948717948718, Math.toRadians(90));
    private final Pose parkControl = new Pose(36.91779788838613, 85.50075414781297);

    private final Pose extraApproachPose = new Pose(15.49778469079939, 91.19909502262443, Math.toRadians(166));

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
    @Override protected Pose getExtraIntakeEndPose() { return parkPose; }
}
