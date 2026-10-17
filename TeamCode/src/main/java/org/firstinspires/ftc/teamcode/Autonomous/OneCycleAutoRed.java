package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Essentials.Constants;

@Autonomous(name = "OneCycleAutoRed")
public class OneCycleAutoRed extends OpMode{
    private Follower follower;
    private AutoMechanisms mechanisms;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        // START POSITION_END POSITION
        // DRIVE > MOVEMENT STATE
        // SHOOT > ATTEMPT TO SCORE THE ARTIFACT
        DRIVE_STARTPOS_SHOOT_POS1,
        SHOOT_POS1_INTAKE_POS,
        INTAKE_POS_PARK_POS,
        SHOOT_PRELOAD1,
        INTAKE1,
        DONE,
    }

    PathState pathState;
    // define path and poses
    private final Pose startPose = new Pose(9,25, Math.toRadians(0));
    private final Pose shootPose1 = new Pose(59, 25, Math.toRadians(90));
    private final Pose intakePose = new Pose(8.876319758672755, 8.349924585218675, Math.toRadians(0));
    private final Pose intakeControl = new Pose(98.49396681749627, 13.523378582202092);
    private final Pose parkPose = new Pose(10.923076923076934, 97.920814479638, Math.toRadians(280));
    private final Pose parkControl = new Pose(24.0844645550528, 29.187782805429887);

    private PathChain driveStartPosShootPos1;
    private PathChain driveShootPos1IntakePos;
    private PathChain driveIntakePoseParkPos;

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
        driveIntakePoseParkPos = follower.pathBuilder()
                .addPath(new BezierCurve(intakePose, parkControl, parkPose))
                .setLinearHeadingInterpolation(intakePose.getHeading(), parkPose.getHeading())
                .build();

    }

    public void statePathUpdate() {
        switch(pathState){
            case DRIVE_STARTPOS_SHOOT_POS1:
                follower.followPath(driveStartPosShootPos1, true);
                setPathState(PathState.SHOOT_PRELOAD1);
                telemetry.addLine("Done Start to shoot1");
                break;
            case SHOOT_PRELOAD1:
                //check if follower has done it's path
                if (!follower.isBusy()){
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
                        setPathState(PathState.INTAKE_POS_PARK_POS);
                        telemetry.addLine("Done with intake1");
                    }
                }
                break;
            case INTAKE_POS_PARK_POS:
                follower.followPath(driveIntakePoseParkPos, true);
                setPathState(PathState.DONE);
                telemetry.addLine("Done Intake to park");
                telemetry.addLine("Done with Auto");
                break;
            case DONE:
                mechanisms.stopFlywheel();
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
