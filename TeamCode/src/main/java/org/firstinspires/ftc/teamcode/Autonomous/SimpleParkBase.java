package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Autonomous.Essentials.Constants;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOpData;

import java.util.List;

import static com.pedropathing.api.Paths.line;

// Starts in the corner of the park wall and the side wall facing away from the park wall,
// then drives in a straight line to a park spot PARK_OFF_WALL in off the park wall
public abstract class SimpleParkBase extends OpMode {
    // Measured robot size, inches. The center is 68.627 mm closer to the back than the front
    protected static final double ROBOT_WIDTH = 453.977 / 25.4;
    protected static final double ROBOT_FRONT = (452.627 + 68.62673) / 2 / 25.4;
    protected static final double ROBOT_BACK = (452.627 - 68.62673) / 2 / 25.4;
    // Inner walls are 141.233 in apart, centered in Pedro's 144 in grid
    protected static final double WALL_INSET = (144 - 141.233) / 2;
    // Gap between the robot and the park wall once parked, inches
    protected static final double PARK_OFF_WALL = 3;
    // Park spots along the park wall, measured from the start y
    protected static final double LEFT_PARK_FROM_START = 6.25;
    protected static final double RIGHT_PARK_FROM_START = 35.9551282051282;

    // Blue starts with its back on the x = 144 (park) wall and its side on the y = 0 wall, facing -x
    protected static final Pose BLUE_START = new Pose(
            144 - WALL_INSET - ROBOT_BACK,
            WALL_INSET + ROBOT_WIDTH / 2,
            Math.toRadians(180));
    protected static final Pose BLUE_LEFT_PARK = bluePark(BLUE_START.y() + LEFT_PARK_FROM_START);
    protected static final Pose BLUE_RIGHT_PARK = bluePark(BLUE_START.y() + RIGHT_PARK_FROM_START);
    // Red is Blue rotated 180 deg about (72, 72)
    protected static final Pose RED_START = toRed(BLUE_START);
    protected static final Pose RED_LEFT_PARK = toRed(BLUE_LEFT_PARK);
    protected static final Pose RED_RIGHT_PARK = toRed(BLUE_RIGHT_PARK);

    private static Pose bluePark(double y) {
        return new Pose(BLUE_START.x() - PARK_OFF_WALL, y, Math.toRadians(180));
    }

    private static Pose toRed(Pose p) {
        return new Pose(144 - p.x(), 144 - p.y(), p.heading() - Math.PI);
    }

    private Follower follower;
    private List<LynxModule> hubs;

    protected abstract Pose getStartPose();
    protected abstract Pose getParkPose();

    @Override
    public void init() {
        // MANUAL bulk caching, cleared once at the top of every loop
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(getStartPose());
        TeleOpData.autoEndPose = getStartPose();
    }

    @Override
    public void start() {
        Pose start = getStartPose();
        follower.follow(line(start, getParkPose()).constant(start.heading()));
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
        follower.update();
        Pose pose = follower.pose();
        TeleOpData.autoEndPose = pose;
        telemetry.addData("busy", follower.isBusy());
        telemetry.addData("x", pose.x());
        telemetry.addData("y", pose.y());
        telemetry.addData("Heading (deg)", Math.toDegrees(pose.heading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        // Teleop starts its field position from here
        TeleOpData.autoEndPose = follower.pose();
        follower.stop();
    }
}
