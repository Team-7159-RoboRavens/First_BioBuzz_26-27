package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Autonomous.Essentials.Constants;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

/** Shared Pedro Pathing logic for the one-cycle autonomous routines. */
public abstract class OneCycleAutoBase extends OpMode {
    protected Follower follower;
    protected AutoMechanisms mechanisms;
    protected Timer pathTimer, opModeTimer;

    public enum PathState {
        DRIVE_STARTPOS_SHOOT_POS1,
        SHOOT_POS1_INTAKE_POS,
        INTAKE_POS_PARK_POS,
        SHOOT_PRELOAD1,
        INTAKE1,
        DONE
    }

    protected PathState pathState;

    protected abstract Pose getStartPose();
    protected abstract Pose getShootPose1();
    protected abstract Pose getIntakePose();
    protected abstract Pose getIntakeControl();
    protected abstract Pose getParkPose();
    protected abstract Pose getParkControl();

    protected Path driveStartPosShootPos1;
    protected Path driveShootPos1IntakePos;
    protected Path driveIntakePoseParkPos;

    protected void buildPaths() {
        Pose startPose = getStartPose();
        Pose shootPose1 = getShootPose1();
        Pose intakePose = getIntakePose();
        Pose intakeControl = getIntakeControl();
        Pose parkPose = getParkPose();
        Pose parkControl = getParkControl();

        driveStartPosShootPos1 = line(startPose, shootPose1).linear(startPose, shootPose1);
        driveShootPos1IntakePos = curve(shootPose1, intakeControl, intakePose)
                .linear(shootPose1, intakePose);
        driveIntakePoseParkPos = curve(intakePose, parkControl, parkPose)
                .linear(intakePose, parkPose);
    }

    protected void statePathUpdate() {
        switch (pathState) {
            case DRIVE_STARTPOS_SHOOT_POS1:
                follower.follow(driveStartPosShootPos1);
                setPathState(PathState.SHOOT_PRELOAD1);
                break;
            case SHOOT_PRELOAD1:
                if (!follower.isBusy()) {
                    mechanisms.shoot();
                    if (pathTimer.seconds() >= 0.6) {
                        mechanisms.resetFeeder();
                        setPathState(PathState.SHOOT_POS1_INTAKE_POS);
                    }
                }
                break;
            case SHOOT_POS1_INTAKE_POS:
                follower.follow(driveShootPos1IntakePos);
                setPathState(PathState.INTAKE1);
                break;
            case INTAKE1:
                if (!follower.isBusy()) {
                    mechanisms.intake();
                    if (pathTimer.seconds() >= 0.25) {
                        setPathState(PathState.INTAKE_POS_PARK_POS);
                    }
                }
                break;
            case INTAKE_POS_PARK_POS:
                follower.follow(driveIntakePoseParkPos);
                setPathState(PathState.DONE);
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

    protected void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.reset();
    }

    @Override
    public void init() {
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
    public void start() {
        opModeTimer.reset();
        setPathState(pathState);
    }

    @Override
    public void loop() {
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
