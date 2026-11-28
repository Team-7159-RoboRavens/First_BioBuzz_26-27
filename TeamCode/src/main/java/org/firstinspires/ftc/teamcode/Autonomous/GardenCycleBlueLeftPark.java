package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// Parks left in a straight line from the first shoot pose, see GardenCycleBlueBase
@Autonomous(name = "GardenCycleBlueLeftPark")
public class GardenCycleBlueLeftPark extends GardenCycleBlueBase {
    private final Pose parkPose = new Pose(130.9674, 20.8556, Math.toRadians(180));

    @Override protected Pose getParkPose() { return parkPose; }
}
