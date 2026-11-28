package org.firstinspires.ftc.teamcode.Shooter;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

// Counts balls shot from the flywheel: each ball pulls the RPM well below the target and makes the motor draw more
// current to get back up. Tuned from BallCountDataTest on 2026-11-27: 40 of 40 balls counted, no false counts.
// Real shots dipped 13-33 RPM and drew 1.7-2.8 A, with no ball the RPM moved up to about 10 under and stayed under 1 A
public class ShotCounter {
    // ---------------- Tuning ----------------
    // RPM below the target that starts a possible ball. Lower it if balls are missed, raise it if noise counts.
    // RPM reads in steps of about 3.1, so keep it well above that
    public static double DIP_RPM = 12;
    // RPM has to come back to within this of the target before the next ball can count, keep it below DIP_RPM
    public static double RECOVER_RPM = 6;
    // Shortest time between two counted balls, seconds, so one ball can't count twice
    public static double MIN_SHOT_GAP = 0.15;
    // A dip only counts if the flywheel current reaches this within CONFIRM_TIME, amps. Noise dips stay under 1 A.
    // Only used when the counter has the motor
    public static boolean USE_CURRENT = true;
    public static double CONFIRM_AMPS = 1.4;
    public static double CONFIRM_TIME = 0.3;
    // A ball drops the RPM fast (about 16 RPM in 25 ms). The dip only counts if the RPM was within RECOVER_RPM of the
    // target this recently, seconds, so a slow sag (the controller undershooting) isn't a ball. Kept all 40 of 40
    public static double MAX_DROP_TIME = 0.15;
    // Dips are measured from the target smoothed over this many seconds. Teleop's target can jump 6-10 RPM in one
    // loop, and a jump up looked like a dip (FlywheelControlTest counted one with no balls). 0.2 s still counts 40 of 40
    public static double TARGET_SMOOTHING_TIME = 0.2;
    // ----------------------------------------

    // Null = count on the RPM dip alone
    private final DcMotorEx motor;
    private int shots = 0;
    // RPM is near the target, so the next dip counts
    private boolean armed = false;
    private double lastShotTime = Double.NEGATIVE_INFINITY;
    // Time the current dip started, -1 when not waiting for the current
    private double dipStart = -1;
    private double lastAmps = 0;
    // Last time the RPM was within RECOVER_RPM of the target
    private double lastNearTime = Double.NEGATIVE_INFINITY;
    private double smoothTarget = Double.NaN;
    private double lastTime = Double.NaN;

    public ShotCounter() {
        this(null);
    }

    // Current reads are their own hub command, so they are only read during a dip
    public ShotCounter(DcMotorEx flywheel) {
        motor = flywheel;
    }

    public void reset() {
        shots = 0;
        armed = false;
        dipStart = -1;
        lastShotTime = Double.NEGATIVE_INFINITY;
        smoothTarget = Double.NaN;
        lastTime = Double.NaN;
    }

    // Call every loop while feeding. atSpeed false (still spinning up) never counts a ball
    public void update(double rpm, double rawTargetRPM, double time, boolean atSpeed) {
        if (Double.isNaN(smoothTarget) || rawTargetRPM <= 0 || TARGET_SMOOTHING_TIME <= 0) {
            smoothTarget = rawTargetRPM;
        } else {
            double dt = Math.max(0, time - lastTime);
            smoothTarget += (1 - Math.exp(-dt / TARGET_SMOOTHING_TIME)) * (rawTargetRPM - smoothTarget);
        }
        lastTime = time;
        double targetRPM = smoothTarget;
        if (!atSpeed || targetRPM <= 0) {
            armed = false;
            dipStart = -1;
            return;
        }
        if (rpm >= targetRPM - RECOVER_RPM) {
            lastNearTime = time;
        }
        if (dipStart >= 0) {
            if (confirmed()) {
                shots++;
                lastShotTime = time;
                dipStart = -1;
            } else if (time - dipStart > CONFIRM_TIME) {
                // Just noise
                dipStart = -1;
            }
            return;
        }
        if (!armed) {
            if (rpm >= targetRPM - RECOVER_RPM && time - lastShotTime >= MIN_SHOT_GAP) {
                armed = true;
            }
            return;
        }
        if (rpm < targetRPM - DIP_RPM) {
            armed = false;
            if (time - lastNearTime > MAX_DROP_TIME) {
                // Sagged slowly, not a ball
                return;
            }
            dipStart = time;
            if (confirmed()) {
                shots++;
                lastShotTime = time;
                dipStart = -1;
            }
        }
    }

    private boolean confirmed() {
        if (motor == null || !USE_CURRENT) {
            return true;
        }
        lastAmps = motor.getCurrent(CurrentUnit.AMPS);
        return lastAmps >= CONFIRM_AMPS;
    }

    public int getShots() {
        return shots;
    }

    // Last flywheel current read during a dip, amps
    public double getLastAmps() {
        return lastAmps;
    }
}
