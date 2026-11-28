package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// Parks left in a straight line from the first shoot pose, see GardenCycleRedBase
@Autonomous(name = "GardenCycleRedLeftPark")
public class GardenCycleRedLeftPark extends GardenCycleRedBase {
    private final Pose parkPose = new Pose(13.0326, 121.3144, Math.toRadians(0));

    @Override protected Pose getParkPose() { return parkPose; }
}
