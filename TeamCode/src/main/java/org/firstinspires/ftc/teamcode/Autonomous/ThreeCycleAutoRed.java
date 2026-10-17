package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "ThreeCycleAutoRed", group = "Autonomous")
public class ThreeCycleAutoRed extends ThreeCycleBase { // Extends the base class

    @Override
    public void init() {
        // 1. Assign your exact Red specific coordinate positions
        startPose = new Pose(9, 25, Math.toRadians(0));
        shootPose1 = new Pose(59, 25, Math.toRadians(90));
        intakePose = new Pose(9, 8.3499, Math.toRadians(0));
        intakeControl = new Pose(98.4939, 13.5233);
        shootPose2 = new Pose(49.0874, 131.7225, Math.toRadians(280));
        shoot2Control = new Pose(45.0, 96.9758);
        parkPose = new Pose(9.8559, 112.2202, Math.toRadians(280));
        parkControl = new Pose(45.8536, 110.2888);

        // 2. Call the base class initialization to map hardware maps and systems
        super.init();

        // 3. Initialize follower coordinates and compile the chains
        follower.setStartingPose(startPose);
        follower.setPose(startPose);
        buildPaths();
        setPathState(PathState.DRIVE_STARTPOS_SHOOT_POS1);
    }
}
