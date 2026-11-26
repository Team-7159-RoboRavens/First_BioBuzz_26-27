package org.firstinspires.ftc.teamcode;

// Sees a ball entering from the intake motor slowing down. Speed is divided by power so the analog trigger doesn't
// look like a ball. Thresholds are guesses until tuned from the BallCountDataTest intake_tps data
public class IntakeBallDetector {
    // ---------------- Tuning ----------------
    // Off until the thresholds below are tuned, the count then only changes from shots and g2
    public static boolean ENABLED = false;
    // Speed per power this far below the running average counts as a ball (0.25 = 25% slower)
    public static double DIP_FRACTION = 0.25;
    // Speed has to come back to within this of the average before the next ball can count
    public static double RECOVER_FRACTION = 0.10;
    // The dip has to last this long, seconds, so one noisy reading doesn't count
    public static double MIN_DIP_TIME = 0.03;
    // Shortest time between two counted balls, seconds
    public static double MIN_BALL_GAP = 0.15;
    // Spin up after the intake starts (or after a jam reverse) is ignored for this long, seconds
    public static double START_IGNORE_TIME = 0.4;
    // Smoothing time of the running average speed with no ball in it, seconds
    public static double BASELINE_SMOOTHING_TIME = 0.5;
    // Powers below this are not watched
    public static double MIN_POWER = 0.2;
    // ----------------------------------------

    private boolean running = false;
    private boolean armed = true;
    private double baseline = Double.NaN;
    private double ignoreUntil = 0;
    private double dipStart = -1;
    private double lastBallTime = Double.NEGATIVE_INFINITY;
    private double lastTime = Double.NaN;
    private int balls = 0;

    // Call every loop with the power sent to the intake and its speed (any unit), returns true when a ball came in
    public boolean update(double power, double speed, boolean reversing, double now) {
        double dt = Double.isNaN(lastTime) ? 0 : Math.max(0, now - lastTime);
        lastTime = now;
        if (!ENABLED || power < MIN_POWER || reversing) {
            running = false;
            return false;
        }
        double ratio = Math.abs(speed) / power;
        if (!running) {
            running = true;
            armed = true;
            dipStart = -1;
            baseline = ratio;
            ignoreUntil = now + START_IGNORE_TIME;
        }
        if (now < ignoreUntil) {
            // Follows the spin up closely so the average is right once watching starts
            baseline = ratio;
            return false;
        }
        if (!armed) {
            if (ratio >= baseline * (1 - RECOVER_FRACTION) && now - lastBallTime >= MIN_BALL_GAP) {
                armed = true;
            }
            return false;
        }
        if (ratio < baseline * (1 - DIP_FRACTION)) {
            if (dipStart < 0) {
                dipStart = now;
            }
            if (now - dipStart >= MIN_DIP_TIME) {
                dipStart = -1;
                armed = false;
                lastBallTime = now;
                balls++;
                return true;
            }
            return false;
        }
        dipStart = -1;
        // Only the speed with no ball in it goes into the average
        double blend = BASELINE_SMOOTHING_TIME <= 0 ? 1 : 1 - Math.exp(-dt / BASELINE_SMOOTHING_TIME);
        baseline += blend * (ratio - baseline);
        return false;
    }

    public double getBaseline() {
        return baseline;
    }

    // Balls seen since the op mode started
    public int getBalls() {
        return balls;
    }
}
