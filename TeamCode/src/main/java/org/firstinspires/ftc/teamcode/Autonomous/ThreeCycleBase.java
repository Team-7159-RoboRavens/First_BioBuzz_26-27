package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Autonomous.Essentials.Constants;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

public abstract class ThreeCycleBase extends OpMode {
    protected Follower follower;
    protected AutoMechanisms mechanisms;
    protected Timer pathTimer, opModeTimer;

    public enum PathState {
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

    protected PathState pathState;

    // Abstract pose getters to be implemented by Red/Blue subclasses
    protected abstract Pose getStartPose();
    protected abstract Pose getShootPose1();
    protected abstract Pose getIntakePose();
    protected abstract Pose getIntakeControl();
    protected abstract Pose getShootPose2();
    protected abstract Pose getShoot2Control();
    protected abstract Pose getParkPose();
    protected abstract Pose getParkControl();

    protected Path driveStartPosShootPos1;
    protected Path driveShootPos1IntakePos;
    protected Path driveIntakePoseShootPos2;
    protected Path driveShootPos2ParkPos;

    public void buildPaths() {
        Pose startPose = getStartPose();
        Pose shootPose1 = getShootPose1();
        Pose intakePose = getIntakePose();
        Pose intakeControl = getIntakeControl();
        Pose shootPose2 = getShootPose2();
        Pose shoot2Control = getShoot2Control();
        Pose parkPose = getParkPose();
        Pose parkControl = getParkControl();

        driveStartPosShootPos1 = line(startPose, shootPose1).linear(startPose, shootPose1);
        driveShootPos1IntakePos = curve(shootPose1, intakeControl, intakePose).linear(shootPose1, intakePose);
        driveIntakePoseShootPos2 = curve(intakePose, shoot2Control, shootPose2).linear(intakePose, shootPose2);
        driveShootPos2ParkPos = curve(shootPose2, parkControl, parkPose).linear(shootPose2, parkPose);
    }

    public void statePathUpdate() {
        switch(pathState){
            case DRIVE_STARTPOS_SHOOT_POS1:
                follower.follow(driveStartPosShootPos1);
                setPathState(PathState.SHOOT_PRELOAD1);
                telemetry.addLine("Done with Path 1");
                break;

            case SHOOT_PRELOAD1:
                telemetry.addLine("Done Start to shoot1");
                if (!follower.isBusy()){
                    telemetry.addLine("Follower is not busy");
                    mechanisms.shoot();

                    if (pathTimer.seconds() >= 0.6) {
                        mechanisms.resetFeeder();
                        setPathState(PathState.SHOOT_POS1_INTAKE_POS);
                        telemetry.addLine("Done Shooting 1");
                    }
                }
                break;
            case SHOOT_POS1_INTAKE_POS:
                follower.follow(driveShootPos1IntakePos);
                setPathState(PathState.INTAKE1);
                telemetry.addLine("Done Shoot1 to intake");
                break;
            case INTAKE1:
                if(!follower.isBusy()){
                    mechanisms.intake();
                    if (pathTimer.seconds() >= 0.25) {
                        setPathState(PathState.INTAKE_POS_SHOOT_POS2);
                        telemetry.addLine("Done with intake1");
                    }
                }
                break;
            case INTAKE_POS_SHOOT_POS2:
                follower.follow(driveIntakePoseShootPos2);
                setPathState(PathState.SHOOT_PRELOAD2);
                telemetry.addLine("Done Intake to shoot2");
                break;
            case SHOOT_PRELOAD2:
                if (!follower.isBusy()){
                    mechanisms.shoot();
                    if (pathTimer.seconds() >= 0.6) {
                        mechanisms.resetFeeder();
                        setPathState(PathState.INTAKE2);
                        telemetry.addLine("Done Shooting 2");
                    }
                }
                break;
            case INTAKE2:
                if(!follower.isBusy()){
                    mechanisms.intake();
                    if (pathTimer.seconds() >= 0.25) {
                        setPathState(PathState.SHOOT_PRELOAD3);
                        telemetry.addLine("Done with intake2");
                    }
                }
                break;
            case SHOOT_PRELOAD3:
                if (!follower.isBusy()){
                    mechanisms.shoot();
                    if (pathTimer.seconds() >= 0.6) {
                        mechanisms.resetFeeder();
                        setPathState(PathState.SHOOT_POS2_PARK_POS);
                        telemetry.addLine("Done Shooting 3");
                    }
                }
                break;
            case SHOOT_POS2_PARK_POS:
                follower.follow(driveShootPos2ParkPos);
                setPathState(PathState.DONE);
                telemetry.addLine("Done Shoot2 to park");
                telemetry.addLine("Done with Auto");
                break;
            case DONE:
                // Do not stop immediately after starting the park path. Wait until
                // Pedro has finished the final path, then hold the robot still.
                if (!follower.isBusy()) {
                    mechanisms.reset();
                    follower.stop();
                }
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.reset();
    }

    @Override
    public void init(){
        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS1;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.reset();
        follower = Constants.createFollower(hardwareMap);
        mechanisms = new AutoMechanisms(hardwareMap);
        follower.setPose(getStartPose());
        buildPaths();
    }

    @Override
    public void start(){
        opModeTimer.reset();
        setPathState(pathState);
    }

    @Override
    public void loop(){
        follower.update();
        statePathUpdate();

        telemetry.addData("Path State", pathState.toString());
        telemetry.addData("x", follower.pose().x());
        telemetry.addData("y", follower.pose().y());
        telemetry.addData("Heading", follower.pose().heading());
        telemetry.addData("Path Time", pathTimer.seconds());
        telemetry.update();
    }
}