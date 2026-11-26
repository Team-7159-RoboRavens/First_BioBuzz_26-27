package org.firstinspires.ftc.teamcode.Shooter;

// Counts balls shot from the flywheel RPM: each ball pulls the RPM down below the target, then it comes back up.
// Tune from the "ShotRPM" logcat lines the auto writes while feeding
public class ShotCounter {
    // ---------------- Tuning ----------------
    // RPM below the target that counts as a ball going through. Lower it if balls are missed, raise it if noise counts
    public static double DIP_RPM = 3;
    // RPM has to come back to within this of the target before the next ball can count. Keep it below DIP_RPM,
    // or RPM noise right at the line counts as balls
    public static double RECOVER_RPM = 1.5;
    // Shortest time between two counted balls, seconds, so one ball can't count twice
    public static double MIN_SHOT_GAP = 0.12;
    // ----------------------------------------

    private int shots = 0;
    // RPM is near the target, so the next dip counts
    private boolean armed = false;
    private double lastShotTime = Double.NEGATIVE_INFINITY;

    public void reset() {
        shots = 0;
        armed = false;
        lastShotTime = Double.NEGATIVE_INFINITY;
    }

    // Call every loop while feeding. atSpeed false (still spinning up) never counts a ball
    public void update(double rpm, double targetRPM, double time, boolean atSpeed) {
        if (!atSpeed || targetRPM <= 0) {
            armed = false;
            return;
        }
        if (!armed) {
            if (rpm >= targetRPM - RECOVER_RPM && time - lastShotTime >= MIN_SHOT_GAP) {
                armed = true;
            }
            return;
        }
        if (rpm < targetRPM - DIP_RPM) {
            shots++;
            lastShotTime = time;
            armed = false;
        }
    }

    public int getShots() {
        return shots;
    }
}
