package org.firstinspires.ftc.teamcode.Autonomous;

// Correct Pedro Pathing 3.0 Production Imports
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;                     // Relocated in 3.0
import com.pedropathing.paths.Path;                   // Replaces PathChain in 3.0
import static com.pedropathing.paths.Paths.path;      // Static 3.0 inline path builders
import static com.pedropathing.paths.Paths.line;
import static com.pedropathing.paths.Paths.curve;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import android.util.Log;

import org.firstinspires.ftc.teamcode.Autonomous.Essentials.Constants;

@Autonomous(name = "ThreeCycleAutoRed", group = "Autonomous")
public class ThreeCycleAutoRed extends OpMode {
    private Follower follower;
    private AutoMechanisms mechanisms;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        DRIVE_STARTPOS_SHOOT_POS1,
        SHOOT_PRELOAD1,
        DRIVE_SHOOT_POS1_INTAKE_POS,
        INTAKE1,
        DRIVE_INTAKE_POS_SHOOT_POS2,
        SHOOT_PRELOAD2,
        INTAKE2,
        SHOOT_PRELOAD3,
        DRIVE_SHOOT_POS2_PARK_POS,
        DONE
    }

    private PathState pathState;

    // Define coordinate Poses (Heading field values remain the same)
    private final Pose startPose = new Pose(9, 25, Math.toRadians(0));
    private final Pose shootPose1 = new Pose(59, 25, Math.toRadians(90));

    private final Pose intakePose = new Pose(9, 8.349924585218675, Math.toRadians(0));
    private final Pose intakeControl = new Pose(98.49396681749627, 13.523378582202092);

    private final Pose shootPose2 = new Pose(49.08748114630466, 131.72252073906486, Math.toRadians(280));
    private final Pose shoot2Control = new Pose(45.0, 96.97586726998493);

    private final Pose parkPose = new Pose(9.855957767722485, 112.22021116138762, Math.toRadians(280));
    private final Pose parkControl = new Pose(45.853695324283564, 110.28883861236804);

    // CHANGED: Replacing legacy PathChain arrays with modern Pedro 3.0 Path instances
    private Path driveStartPosShootPos1;
    private Path driveShootPos1IntakePos;
    private Path driveIntakePoseShootPos2;
    private Path driveShootPos2ParkPos;

    public void buildPaths() {
        // FIXED: Replaced old .pathBuilder() chains with the fast 3.0 static builders
        // FIXED: Replaced .getHeading() calls with native .heading accessor properties
        driveStartPosShootPos1 = path(
                line(startPose, shootPose1)
                        .linear(startPose.heading, shootPose1.heading)
        );

        driveShootPos1IntakePos = path(
                curve(shootPose1, intakeControl, intakePose)
                        .linear(shootPose1.heading, intakePose.heading)
        );

        driveIntakePoseShootPos2 = path(
                curve(intakePose, shoot2Control, shootPose2)
                        .linear(intakePose.heading, shootPose2.heading)
        );

        driveShootPos2ParkPos = path(
                curve(shootPose2, parkControl, parkPose)
                        .linear(shootPose2.heading, parkPose.heading)
        );
    }

    public void statePathUpdate() {
        // Continuous position logging to Android Studio Logcat
        // FIXED: Replaced .getPose() with .pose(), and changed math property coordinates
        Pose currentPose = follower.pose();
        Log.i("RobotPosition", String.format("State: %s | X: %.2f, Y: %.2f, Heading: %.2f",
                pathState.toString(), currentPose.x, currentPose.y, Math.toDegrees(currentPose.heading)));

        switch (pathState) {
            case DRIVE_STARTPOS_SHOOT_POS1:
                if (!follower.isBusy()) {
                    follower.follow(driveStartPosShootPos1, true); // FIXED: .follow() replaces .followPath()
                }

                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_PRELOAD1);
                }
                break;

            case SHOOT_PRELOAD1:
                // FIXED: Timer tracks seconds via .seconds() in 3.0
                if (pathTimer.seconds() > 0.1) {
                    mechanisms.shoot();

                    if (pathTimer.seconds() >= 1.2) {
                        mechanisms.reset();
                        setPathState(PathState.DRIVE_SHOOT_POS1_INTAKE_POS);
                    }
                }
                break;

            case DRIVE_SHOOT_POS1_INTAKE_POS:
                if (!follower.isBusy()) {
                    follower.follow(driveShootPos1IntakePos, true);
                }

                if (!follower.isBusy()) {
                    setPathState(PathState.INTAKE1);
                }
                break;

            case INTAKE1:
                if (pathTimer.seconds() > 0.1) {
                    mechanisms.intake();
                    if (pathTimer.seconds() >= 1.0) {
                        mechanisms.stopIntake();
                        setPathState(PathState.DRIVE_INTAKE_POS_SHOOT_POS2);
                    }
                }
                break;

            case DRIVE_INTAKE_POS_SHOOT_POS2:
                if (!follower.isBusy()) {
                    follower.follow(driveIntakePoseShootPos2, true);
                }

                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_PRELOAD2);
                }
                break;

            case SHOOT_PRELOAD2:
                if (pathTimer.seconds() > 0.1) {
                    mechanisms.shoot();
                    if (pathTimer.seconds() >= 1.2) {
                        mechanisms.reset();
                        setPathState(PathState.INTAKE2);
                    }
                }
                break;

            case INTAKE2:
                if (pathTimer.seconds() > 0.1) {
                    mechanisms.intake();
                    if (pathTimer.seconds() >= 1.0) {
                        mechanisms.stopIntake();
                        setPathState(PathState.SHOOT_PRELOAD3);
                    }
                }
                break;

            case SHOOT_PRELOAD3:
                if (pathTimer.seconds() > 0.1) {
                    mechanisms.shoot();
                    if (pathTimer.seconds() >= 1.2) {
                        mechanisms.reset();
                        setPathState(PathState.DRIVE_SHOOT_POS2_PARK_POS);
                    }
                }
                break;

            case DRIVE_SHOOT_POS2_PARK_POS:
                if (!follower.isBusy()) {
                    follower.follow(driveShootPos2ParkPos, true);
                }

                if (!follower.isBusy()) {
                    setPathState(PathState.DONE);
                }
                break;

            case DONE:
                if (!follower.isBusy()) {
                    mechanisms.reset();
                }
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.reset(); // FIXED: Timer resets via .reset() instead of resetTimer()
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.reset();

        // FIXED: This assumes your Constants file has been updated to provide the 3.0 factory setup.
        // If Constants.java is red, you will need to open it and use: new Follower(hardwareMap)
        follower = Constants.createFollower(hardwareMap);
        mechanisms = new AutoMechanisms(hardwareMap);

        // FIXED: Replaced legacy setStartingPose method with direct assignment or setPose
        follower.setPose(startPose);
        buildPaths();
        setPathState(PathState.DRIVE_STARTPOS_SHOOT_POS1);
    }

    @Override
    public void start() {
        opModeTimer.reset();
        pathTimer.reset();
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        Pose currentPose = follower.pose(); // FIXED: Access current pose with .pose()
        telemetry.addData("Path State", pathState.toString());
        telemetry.addData("x", currentPose.x);
        telemetry.addData("y", currentPose.y);
        telemetry.addData("Heading (Deg)", Math.toDegrees(currentPose.heading));
        telemetry.update();
    }
}
