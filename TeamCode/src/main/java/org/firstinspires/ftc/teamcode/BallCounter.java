package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

// Keeps how many balls are in the robot: +1 from the intake, -1 per shot, g2 can fix it by hand.
// g1 buzzes when it drops to 0, g2 buzzes and the intake stops when it fills up from intaking.
// A ball in while full buzzes g2 twice and doesn't stop the intake. The count stays at MAX_BALLS until g2 raises it
// past MAX_BALLS by hand, after that it has no top and the intake never stops, until g2 presses dpad left at 0
public class BallCounter {
    // ---------------- Tuning ----------------
    public static int MAX_BALLS = 4;
    public static int RUMBLE_MS = 400;
    // ----------------------------------------

    private final Gamepad g1;
    private final Gamepad g2;
    private int count = 0;
    // Filled up from intaking, the intake stays off until g2 lets go of it or a ball leaves
    private boolean intakeLocked = false;
    // g2 went past MAX_BALLS by hand (the penalty only comes once), no top until g2 presses dpad left at 0
    private boolean capRemoved = false;

    public BallCounter(Gamepad g1, Gamepad g2) {
        this.g1 = g1;
        this.g2 = g2;
    }

    // Call every loop. intakeHeld = g2 is asking to intake, manualControls false leaves g2's buttons alone
    public void update(boolean intakeHeld, boolean ballEntered, int ballsShot, boolean manualControls) {
        if (manualControls) {
            // g2 dpad right +1, dpad left -1, B = full, A = empty
            if (g2.dpadRightWasPressed()) {
                if (count >= MAX_BALLS) {
                    capRemoved = true;
                    intakeLocked = false;
                }
                setCount(count + 1, false);
            }
            if (g2.dpadLeftWasPressed()) {
                if (count <= 0) {
                    // Count stays at 0, the MAX_BALLS limit comes back
                    capRemoved = false;
                }
                setCount(count - 1, false);
            }
            if (g2.bWasPressed()) {
                setCount(MAX_BALLS, false);
            }
            if (g2.aWasPressed()) {
                setCount(0, false);
            }
        }
        if (ballsShot > 0) {
            setCount(count - ballsShot, false);
        }
        if (!intakeHeld) {
            // Letting go and intaking again runs the intake even when full
            intakeLocked = false;
        }
        if (ballEntered) {
            setCount(count + 1, true);
        }
    }

    private void setCount(int newCount, boolean fromIntake) {
        if (newCount > MAX_BALLS && fromIntake) {
            // Ball in past full: buzz, pause, buzz, the intake keeps running
            g2.runRumbleEffect(new Gamepad.RumbleEffect.Builder()
                    .addStep(1, 1, RUMBLE_MS)
                    .addStep(0, 0, RUMBLE_MS)
                    .addStep(1, 1, RUMBLE_MS)
                    .build());
        }
        newCount = Math.max(0, capRemoved ? newCount : Math.min(MAX_BALLS, newCount));
        if (newCount == count) {
            return;
        }
        int old = count;
        count = newCount;
        if (newCount == 0 && old > 0) {
            g1.rumble(RUMBLE_MS);
        }
        if (newCount == MAX_BALLS && fromIntake) {
            g2.rumble(RUMBLE_MS);
            // With no limit the intake keeps running, it only buzzes
            intakeLocked = !capRemoved;
        }
        if (newCount < MAX_BALLS) {
            // Room again, the intake runs if g2 is still holding it
            intakeLocked = false;
        }
    }

    public boolean isCapRemoved() {
        return capRemoved;
    }

    public int getCount() {
        return count;
    }

    // True while intaking should be blocked (outtaking and shooting still work)
    public boolean isIntakeLocked() {
        return intakeLocked;
    }
}
