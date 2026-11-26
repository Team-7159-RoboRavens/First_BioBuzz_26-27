package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// Parks right in a straight line from the first shoot pose, see GardenCycleRedBase
@Autonomous(name = "GardenCycleRedRightPark")
public class GardenCycleRedRightPark extends GardenCycleRedBase {
    private final Pose parkPose = new Pose(12.4924585218703, 98.7948717948718, Math.toRadians(0));

    @Override protected Pose getParkPose() { return parkPose; }
}
