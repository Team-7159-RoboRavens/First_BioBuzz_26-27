package org.firstinspires.ftc.teamcode.Autonomous;

public class ParkTransfer {
    private enum Stage {
        TO_PARK,
        WAIT_IN_PARK,
        TO_SHOOT
    }

    private final ThreeCycleAutoBase auto;
    private Stage stage = Stage.TO_PARK;

    ParkTransfer(ThreeCycleAutoBase auto) {
        this.auto = auto;
    }

    void reset() {
        stage = Stage.TO_PARK;
    }

    void update() {
        if (stage != Stage.WAIT_IN_PARK
                && auto.opModeTimer.seconds() > ThreeCycleAutoBase.PARK_DEADLINE) {
            auto.mechanisms.intake();
            auto.setPathState(
                    ThreeCycleAutoBase.PathState.SHOOT_POS2_PARK_POS);
            return;
        }

        switch (stage) {
            case TO_PARK:
                if (auto.pathTimer.seconds() > ThreeCycleAutoBase.PRESPIN_AFTER) {
                    auto.primeFlywheel();
                }
                if (auto.pathTimer.seconds() > 2.5) {
                    auto.mechanisms.lowerFlower();
                }
                if (auto.runPath(
                        () -> auto.curveTo(
                                auto.getIntakePose(),
                                auto.getShoot2Control(),
                                auto.getParkPose()),
                        Double.POSITIVE_INFINITY)) {

                    auto.mechanisms.lowerFlower();
                    auto.follower.hold(auto.getParkPose());
                    stage = Stage.WAIT_IN_PARK;
                    auto.setPathState(
                            ThreeCycleAutoBase.PathState.INTAKE_POS_SHOOT_POS2);
                }
                break;

            case WAIT_IN_PARK:
                if (auto.pathTimer.seconds() >= 0.2) {
                    stage = Stage.TO_SHOOT;
                    auto.setPathState(
                            ThreeCycleAutoBase.PathState.INTAKE_POS_SHOOT_POS2);
                }
                break;

            case TO_SHOOT:
                if (auto.pathTimer.seconds() > ThreeCycleAutoBase.PRESPIN_AFTER) {
                    auto.primeFlywheel();
                }
                if (auto.pathTimer.seconds() > 2.5) {
                    auto.mechanisms.lowerFlower();
                }
                if (auto.runPath(
                        () -> auto.lineTo(
                                auto.getParkPose(),
                                auto.getPreShootPose2()),
                        ThreeCycleAutoBase.INTAKE_TO_SHOOT2_TIMEOUT)) {

                    auto.mechanisms.lowerFlower();
                    auto.setPathState(
                            ThreeCycleAutoBase.PathState.SHOOT_PRELOAD2);
                }
                break;
        }
    }
}