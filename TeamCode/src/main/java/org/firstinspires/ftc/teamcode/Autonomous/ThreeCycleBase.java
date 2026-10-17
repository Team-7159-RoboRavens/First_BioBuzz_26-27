package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import android.util.Log;

import org.firstinspires.ftc.teamcode.Autonomous.Essentials.Constants;

public abstract class ThreeCycleBase extends OpMode {
    protected Follower follower;
    protected AutoMechanisms mechanisms;
    protected Timer pathTimer, opModeTimer;

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

    protected PathState pathState;

    // NEW CRITICAL SAFETY FEATURE: Prevents the path from continuously re-submitting and stuttering
    private boolean pathSubmitted = false;

    protected Pose startPose;
    protected Pose shootPose1;
    protected Pose intakePose;
    protected Pose intakeControl;
    protected Pose shootPose2;
    protected Pose shoot2Control;
    protected Pose parkPose;
    protected Pose parkControl;

    private PathChain driveStartPosShootPos1;
    private PathChain driveShootPos1IntakePos;
    private PathChain driveIntakePoseShootPos2;
    private PathChain driveShootPos2ParkPos;

    protected void buildPaths() {
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
        Pose currentPose = follower.getPose();
        Log.i("RobotPosition", String.format("State: %s | X: %.2f, Y: %.2f, Heading: %.2f",
                pathState.toString(), currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading())));

        switch (pathState) {
            case DRIVE_STARTPOS_SHOOT_POS1:
                // FIX: If the path hasn't been submitted yet, send it once and lock it
                if (!pathSubmitted) {
                    follower.followPath(driveStartPosShootPos1, true);
                    pathSubmitted = true;
                }
                // FIX: Use atParametricEnd() to determine when the robot arrived at its coordinates safely
                if (follower.atParametricEnd() || (pathTimer.getElapsedTimeSeconds() > 0.5 && !follower.isBusy())) {
                    setPathState(PathState.SHOOT_PRELOAD1);
                }
                break;

            case SHOOT_PRELOAD1:
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    mechanisms.shoot();
                    if (pathTimer.getElapsedTimeSeconds() >= 1.2) {
                        mechanisms.reset();
                        setPathState(PathState.DRIVE_SHOOT_POS1_INTAKE_POS);
                    }
                }
                break;

            case DRIVE_SHOOT_POS1_INTAKE_POS:
                if (!pathSubmitted) {
                    follower.followPath(driveShootPos1IntakePos, true);
                    pathSubmitted = true;
                }
                if (follower.atParametricEnd() || (pathTimer.getElapsedTimeSeconds() > 0.5 && !follower.isBusy())) {
                    setPathState(PathState.INTAKE1);
                }
                break;

            case INTAKE1:
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    mechanisms.intake();
                    if (pathTimer.getElapsedTimeSeconds() >= 1.0) {
                        mechanisms.stopIntake();
                        setPathState(PathState.DRIVE_INTAKE_POS_SHOOT_POS2);
                    }
                }
                break;

            case DRIVE_INTAKE_POS_SHOOT_POS2:
                if (!pathSubmitted) {
                    follower.followPath(driveIntakePoseShootPos2, true);
                    pathSubmitted = true;
                }
                if (follower.atParametricEnd() || (pathTimer.getElapsedTimeSeconds() > 0.5 && !follower.isBusy())) {
                    setPathState(PathState.SHOOT_PRELOAD2);
                }
                break;

            case SHOOT_PRELOAD2:
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    mechanisms.shoot();
                    if (pathTimer.getElapsedTimeSeconds() >= 1.2) {
                        mechanisms.reset();
                        setPathState(PathState.INTAKE2);
                    }
                }
                break;

            case INTAKE2:
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    mechanisms.intake();
                    if (pathTimer.getElapsedTimeSeconds() >= 1.0) {
                        mechanisms.stopIntake();
                        setPathState(PathState.SHOOT_PRELOAD3);
                    }
                }
                break;

            case SHOOT_PRELOAD3:
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    mechanisms.shoot();
                    if (pathTimer.getElapsedTimeSeconds() >= 1.2) {
                        mechanisms.reset();
                        setPathState(PathState.DRIVE_SHOOT_POS2_PARK_POS);
                    }
                }
                break;

            case DRIVE_SHOOT_POS2_PARK_POS:
                if (!pathSubmitted) {
                    follower.followPath(driveShootPos2ParkPos, true);
                    pathSubmitted = true;
                }
                if (follower.atParametricEnd() || (pathTimer.getElapsedTimeSeconds() > 0.5 && !follower.isBusy())) {
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
        pathTimer.resetTimer();
        pathSubmitted = false; // Reset the submission flag every time we move to a new task step!
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        mechanisms = new AutoMechanisms(hardwareMap);
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        pathTimer.resetTimer();
        pathSubmitted = false;
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        Pose currentPose = follower.getPose();
        telemetry.addData("Path State", pathState.toString());
        telemetry.addData("x", currentPose.getX());
        telemetry.addData("y", currentPose.getY());
        telemetry.addData("Heading (Deg)", Math.toDegrees(currentPose.getHeading()));
        telemetry.update();
    }
}
