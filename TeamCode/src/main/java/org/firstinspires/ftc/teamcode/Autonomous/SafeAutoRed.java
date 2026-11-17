package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "SafeAutoRed")
public class SafeAutoRed extends ThreeCycleRedRightPark {
    private final ParkTransfer parkTransfer = new ParkTransfer(this);

    @Override
    public void start() {
        parkTransfer.reset();
        super.start();
    }

    @Override
    public void statePathUpdate() {
        if (pathState == PathState.INTAKE_POS_SHOOT_POS2) {
            parkTransfer.update();
        } else {
            super.statePathUpdate();
        }
    }

    @Override
    public void stop() {
        super.stop();
        follower.drivetrain.stop();
    }
}