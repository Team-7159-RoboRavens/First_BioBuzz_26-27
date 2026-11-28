package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// Parks right in a straight line from the first shoot pose, see GardenCycleBlueBase
@Autonomous(name = "GardenCycleBlueRightPark")
public class GardenCycleBlueRightPark extends GardenCycleBlueBase {
    private final Pose parkPose = new Pose(130.9674, 50.5607, Math.toRadians(180));

    @Override protected Pose getParkPose() { return parkPose; }
}
