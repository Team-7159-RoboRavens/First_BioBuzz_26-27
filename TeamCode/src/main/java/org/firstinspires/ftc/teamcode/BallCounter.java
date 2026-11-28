package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

// Keeps how many balls are in the robot. Only g2 adds balls (by hand), shots take them away automatically
// (ShotCounter counted 88 of 88 in testing). g1 buzzes when the count drops to 0, time to go intake.
// The count stays at MAX_BALLS until g2 raises it past MAX_BALLS by hand, after that it has no top
// until g2 presses dpad left at 0
public class BallCounter {
    // ---------------- Tuning ----------------
    public static int MAX_BALLS = 4;
    public static int RUMBLE_MS = 400;
    // ----------------------------------------

    private final Gamepad g1;
    private final Gamepad g2;
    private int count = 0;
    // g2 went past MAX_BALLS by hand, no top until g2 presses dpad left at 0
    private boolean capRemoved = false;

    public BallCounter(Gamepad g1, Gamepad g2) {
        this.g1 = g1;
        this.g2 = g2;
    }

    // Call every loop. manualControls false leaves g2's buttons alone
    public void update(int ballsShot, boolean manualControls) {
        if (manualControls) {
            // g2 dpad right +1, dpad left -1, B = full, A = empty
            if (g2.dpadRightWasPressed()) {
                if (count >= MAX_BALLS) {
                    capRemoved = true;
                }
                setCount(count + 1);
            }
            if (g2.dpadLeftWasPressed()) {
                if (count <= 0) {
                    // Count stays at 0, the MAX_BALLS limit comes back
                    capRemoved = false;
                }
                setCount(count - 1);
            }
            if (g2.bWasPressed()) {
                setCount(MAX_BALLS);
            }
            if (g2.aWasPressed()) {
                setCount(0);
            }
        }
        if (ballsShot > 0) {
            setCount(count - ballsShot);
        }
    }

    private void setCount(int newCount) {
        newCount = Math.max(0, capRemoved ? newCount : Math.min(MAX_BALLS, newCount));
        if (newCount == count) {
            return;
        }
        int old = count;
        count = newCount;
        if (newCount == 0 && old > 0) {
            g1.rumble(RUMBLE_MS);
        }
    }

    public boolean isCapRemoved() {
        return capRemoved;
    }

    public int getCount() {
        return count;
    }
}
