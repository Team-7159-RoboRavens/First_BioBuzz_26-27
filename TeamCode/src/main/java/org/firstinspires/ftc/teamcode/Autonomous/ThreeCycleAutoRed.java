package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.Mecanum;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Essentials.Constants;

@Autonomous(name = "ThreeCycleAutoRed")
public class ThreeCycleAutoRed extends OpMode{
    private Follower follower;
    private AutoMechanisms mechanisms;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        // START POSITION_END POSITION
        // DRIVE > MOVEMENT STATE
        // SHOOT > ATTEMPT TO SCORE THE ARTIFACT
        DRIVE_STARTPOS_SHOOT_POS1,
        SHOOT_POS1_INTAKE_POS,
        INTAKE_POS_SHOOT_POS2,
        SHOOT_POS2_PARK_POS,
        SHOOT_PRELOAD1,
        SHOOT_PRELOAD2,
        SHOOT_PRELOAD3,
        INTAKE1,
        INTAKE2,
        DONE
    }

    PathState pathState;
    // define path and poses
    private final Pose startPose = new Pose(9,25, Math.toRadians(0));
    private final Pose shootPose1 = new Pose(59, 25, Math.toRadians(90));
    private final Pose intakePose = new Pose(8.876319758672755, 8.349924585218675, Math.toRadians(0));
    private final Pose intakeControl = new Pose(98.49396681749627, 13.523378582202092);
    private final Pose shootPose2 = new Pose(49.08748114630466, 131.72252073906486, Math.toRadians(280));
    private final Pose shoot2Control = new Pose(31.91251885369533, 96.97586726998493);
    private final Pose parkPose = new Pose(9.855957767722485, 112.22021116138762, Math.toRadians(280));
    private final Pose parkControl = new Pose(45.853695324283564, 110.28883861236804);

    private PathChain driveStartPosShootPos1;
    private PathChain driveShootPos1IntakePos;
    private PathChain driveIntakePoseShootPos2;
    private PathChain driveShootPos2ParkPos;

    public void buildPaths(){
        //put in coordinates for starting pose > ending pose
        driveStartPosShootPos1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose1.getHeading())
                .build();

        driveShootPos1IntakePos = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose1, intakeControl, intakePose))
                .setLinearHeadingInterpolation(shootPose1.getHeading(), intakePose.getHeading())
                .build();
        driveIntakePoseShootPos2 = follower.pathBuilder()
                .addPath(new BezierCurve(intakePose, shoot2Control, shootPose2))
                .setLinearHeadingInterpolation(intakePose.getHeading(), shootPose2.getHeading())
                .build();
        driveShootPos2ParkPos = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose2, parkControl, parkPose))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), parkPose.getHeading())
                .build();

    }

    public void statePathUpdate() {
        switch(pathState){
            case DRIVE_STARTPOS_SHOOT_POS1:
                follower.followPath(driveStartPosShootPos1, true);
                telemetry.addLine("Done with Path 1");
                setPathState(PathState.SHOOT_PRELOAD1);
                break;

            case SHOOT_PRELOAD1:
                //check if follower has done it's path
                telemetry.addLine("Done Start to shoot1");
                if (!follower.isBusy()){
                    telemetry.addLine("Follower is not busy");
                    mechanisms.shoot();

                    if (pathTimer.getElapsedTimeSeconds() >= 0.6) {
                        mechanisms.resetFeeder();
                        setPathState(PathState.SHOOT_POS1_INTAKE_POS);
                        telemetry.addLine("Done Shooting 1");
                    }
                }
                break;
            case SHOOT_POS1_INTAKE_POS:
                follower.followPath(driveShootPos1IntakePos, true);
                setPathState(PathState.INTAKE1);
                telemetry.addLine("Done Shoot1 to intake");
                break;
            case INTAKE1:
                //check if follower has done it's path
                if(!follower.isBusy()){
                    mechanisms.intake();
                    if (pathTimer.getElapsedTimeSeconds() >= 0.25) {
                        setPathState(PathState.INTAKE_POS_SHOOT_POS2);
                        telemetry.addLine("Done with intake1");
                    }
                }
                break;
            case INTAKE_POS_SHOOT_POS2:
                follower.followPath(driveIntakePoseShootPos2, true);
                setPathState(PathState.SHOOT_PRELOAD2);
                telemetry.addLine("Done Intake to shoot2");
                break;
            case SHOOT_PRELOAD2:
                //check if follower has done it's path
                if (!follower.isBusy()){
                    mechanisms.shoot();
                    if (pathTimer.getElapsedTimeSeconds() >= 0.6) {
                        mechanisms.resetFeeder();
                        setPathState(PathState.INTAKE2);
                        telemetry.addLine("Done Shooting 2");
                    }
                }
                break;
            case INTAKE2:
                //check if follower has done it's path
                if(!follower.isBusy()){
                    mechanisms.intake();
                    if (pathTimer.getElapsedTimeSeconds() >= 0.25) {
                        setPathState(PathState.SHOOT_PRELOAD3);
                        telemetry.addLine("Done with intake2");
                    }
                }
                break;
            case SHOOT_PRELOAD3:
                //check if follower has done it's path
                if (!follower.isBusy()){
                    mechanisms.shoot();
                    if (pathTimer.getElapsedTimeSeconds() >= 0.6) {
                        mechanisms.resetFeeder();
                        setPathState(PathState.SHOOT_POS2_PARK_POS);
                        telemetry.addLine("Done Shooting 3");
                    }
                }
                break;
            case SHOOT_POS2_PARK_POS:
                follower.followPath(driveShootPos2ParkPos, true);
                setPathState(PathState.DONE);
                telemetry.addLine("Done Shoot2 to park");
                telemetry.addLine("Done with Auto");
                break;
            case DONE:
                mechanisms.reset();
                break;

        }
    }
    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }
    @Override
    public void init(){
        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS1;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        mechanisms = new AutoMechanisms(hardwareMap);
        follower.setPose(startPose);


        buildPaths();
    }
    public void start(){
        opModeTimer.resetTimer();
        setPathState(pathState);
    }
    @Override
    public void loop(){
        follower.update();
        statePathUpdate();

        telemetry.addData("Path State", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());
    }
}
