package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Autonomous.Essentials.Constants;

@TeleOp
public class ThreeCycleAuto extends OpMode{
    private Follower follower;
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
        INTAKE2
    }

    PathState pathState;
    // define path and poses
    private final Pose startPose = new Pose(133,117, Math.toRadians(180));
    private final Pose shootPose1 = new Pose(85.23906485671196, 117, Math.toRadians(270));
    private final Pose intakePose = new Pose(133.08898944193066, 133.62971342383105, Math.toRadians(0));
    private final Pose intakeControl = new Pose(64.13273001508298, 126.42458521870286);
    private final Pose shootPose2 = new Pose(92.19909502262443, 10.49778469079939, Math.toRadians(108));
    private final Pose shoot2Control = new Pose(127.83785822021117,37.63197586726997);
    private final Pose parkPose = new Pose(131.29411764705884, 30.052036199095017, Math.toRadians(108));

    private PathChain driveStartPosShootPos1;
    private PathChain driveShootPos1IntakePos;
    private PathChain driveIntakePoseShootPos2;
    private PathChain driveShootPos2ParkPos;

    public void buildPaths(){
        //put in coordinates for starting pose > ending pose
        driveStartPosShootPos1 = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, intakeControl, shootPose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose1.getHeading())
                .build();

        driveShootPos1IntakePos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose1, intakePose))
                .setLinearHeadingInterpolation(shootPose1.getHeading(), intakePose.getHeading())
                .build();
        driveIntakePoseShootPos2 = follower.pathBuilder()
                .addPath(new BezierCurve(intakePose, shoot2Control, shootPose2))
                .setLinearHeadingInterpolation(intakePose.getHeading(), shootPose2.getHeading())
                .build();
        driveShootPos2ParkPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose2, parkPose))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), parkPose.getHeading())
                .build();

    }

    public void statePathUpdate() {
        switch(pathState){
            case DRIVE_STARTPOS_SHOOT_POS1:
                follower.followPath(driveStartPosShootPos1, true);
                pathState = PathState.SHOOT_PRELOAD1;
                telemetry.addLine("Done Start to shoot1");
                break;
            case SHOOT_PRELOAD1:
                //check if follower has done it's path
                if (!follower.isBusy()){
                    // TODO add logic to flywheel shooter
                    pathState = PathState.SHOOT_POS1_INTAKE_POS;
                    telemetry.addLine("Done Shooting 1");
                }
            case SHOOT_POS1_INTAKE_POS:
                follower.followPath(driveShootPos1IntakePos, true);
                pathState = PathState.INTAKE1;
                telemetry.addLine("Done Shoot1 to intake");
                break;
            case INTAKE1:
                //check if follower has done it's path
                if(!follower.isBusy()){
                    //TODO add logic to intake servo
                    pathState = PathState.INTAKE_POS_SHOOT_POS2;
                    telemetry.addLine("Done with intake1");
                }
            case INTAKE_POS_SHOOT_POS2:
                follower.followPath(driveIntakePoseShootPos2, true);
                pathState = PathState.SHOOT_PRELOAD2;
                telemetry.addLine("Done Intake to shoot2");
                break;
            case SHOOT_PRELOAD2:
                //check if follower has done it's path
                if (!follower.isBusy()){
                    // TODO add logic to flywheel shooter
                    pathState = PathState.INTAKE2;
                    telemetry.addLine("Done Shooting 2");
                }
            case INTAKE2:
                //check if follower has done it's path
                if(!follower.isBusy()){
                    //TODO add logic to intake servo
                    pathState = PathState.SHOOT_PRELOAD3;
                    telemetry.addLine("Done with intake2");
                }
            case SHOOT_PRELOAD3:
                //check if follower has done it's path
                if (!follower.isBusy()){
                    // TODO add logic to flywheel shooter
                    pathState = PathState.SHOOT_POS2_PARK_POS;
                    telemetry.addLine("Done Shooting 2");
                }
            case SHOOT_POS2_PARK_POS:
                follower.followPath(driveShootPos2ParkPos, true);
                telemetry.addLine("Done Shoot2 to park");
                telemetry.addLine("Done with Auto");
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
        // TODO add in any other init mechanisms

        buildPaths();
        follower.setPose(startPose);
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
