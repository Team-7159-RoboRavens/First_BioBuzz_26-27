package org.firstinspires.ftc.teamcode.ButtonMaps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Shooter.ShotCalculator;

// Same as the working bot controls, but g2's dpad sets the RPM by hand:
// up = START_RPM, right = +1, left = -1, down = 0. Shows the distance and both RPMs to find what scores where
public class ShootTestButtonMap extends WorkingBotButtonMap {
    // RPM dpad up goes to, and what it starts at
    public static double START_RPM = 192;
    // How much each dpad right / left press changes the RPM
    public static double RPM_STEP = 1;

    private final Robot robot;
    private final Telemetry telemetry;
    private double testRPM = START_RPM;

    public ShootTestButtonMap(OpMode op, Robot r, boolean red) {
        super(op, r, red);
        robot = r;
        telemetry = op.telemetry;
    }

    // g2 dpad left / right change the RPM here
    @Override
    protected boolean ballCountButtonsEnabled() {
        return false;
    }

    @Override
    protected String getRPMModeName() {
        return "manual (g2 dpad)";
    }

    @Override
    protected double getShooterRPM() {
        if (g2.dpadUpWasPressed()) {
            testRPM = START_RPM;
        }
        if (g2.dpadRightWasPressed()) {
            testRPM += RPM_STEP;
        }
        if (g2.dpadLeftWasPressed()) {
            testRPM = Math.max(0, testRPM - RPM_STEP);
        }
        if (g2.dpadDownWasPressed()) {
            testRPM = 0;
        }

        ShotCalculator.Shot shot = getShot();
        boolean shotValid = shot != null && shot.valid;
        telemetry.addLine("---- SHOOT TEST ----");
        if (shotValid) {
            telemetry.addData("Distance to target", "%.1f in", shot.openingDistance);
        } else {
            telemetry.addData("Distance to target", "unknown (no field position)");
        }
        telemetry.addData("Wheel RPM", "target %.0f, actual %.1f", testRPM, robot.getShootRPM());
        if (shotValid) {
            telemetry.addData("Math RPM for this distance", "%.1f", shot.rpm);
        } else {
            telemetry.addData("Math RPM for this distance", "none");
        }
        return testRPM;
    }
}
