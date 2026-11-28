package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

// Unjams the intake: when its motor current stays high (a stuck ball stalls it), it runs backwards for a moment and
// then goes back to the wanted power. Used by teleop (Robot) and auto (AutoMechanisms)
public class IntakeJamGuard {
    // ---------------- Tuning ----------------
    public static boolean ENABLED = true;
    // Current that counts as stalled, amps, and how long it has to stay above it, seconds. From BallCountDataTest
    // 2026-11-27: stuck balls held 4.5-5 A, above 4.0 A for 0.6-4 s at a time. Normal 1-4 ball feeds peaked at
    // 4.25 A and were never above 4.0 A for more than 0.1 s. The old 6 A was never reached, so it never reversed
    public static double JAM_CURRENT_AMPS = 4.0;
    public static double JAM_TIME = 0.4;
    // How long and how hard it runs backwards
    public static double REVERSE_TIME = 0.25;
    public static double REVERSE_POWER = 0.6;
    // Current spike when the motor starts up is ignored for this long, seconds
    public static double START_IGNORE_TIME = 0.3;
    // Each current read is its own hub command (not bulk cached), so it is only read this often, seconds
    public static double CHECK_EVERY = 0.05;
    // Gives up after this many jams in a row (a full robot also stalls the intake), until the intake is turned off
    public static int MAX_REVERSALS = 3;
    // A jam counts as "in a row" if it comes within this many seconds of the last reverse
    public static double REPEAT_WINDOW = 1.5;
    // Powers below this are not watched
    public static double MIN_POWER = 0.2;
    // ----------------------------------------

    private final DcMotorEx motor;
    private double lastCurrent = 0;
    private int jams = 0;
    private int reversalsInARow = 0;
    private boolean gaveUp = false;
    private double lastPowerSign = 0;
    private double highSince = -1;
    private double reverseUntil = -1;
    private double lastReverseEnd = Double.NEGATIVE_INFINITY;
    private double ignoreUntil = 0;
    private double nextCheck = 0;

    public IntakeJamGuard(DcMotorEx motor) {
        this.motor = motor;
    }

    // Call every loop with the wanted intake power, returns the power to send to the motor
    public double update(double power) {
        double now = System.nanoTime() / 1e9;
        if (!ENABLED || Math.abs(power) < MIN_POWER) {
            // Intake off: forget everything, the next start gets a fresh set of reversals
            lastPowerSign = 0;
            highSince = -1;
            reverseUntil = -1;
            reversalsInARow = 0;
            gaveUp = false;
            return power;
        }
        double sign = Math.signum(power);
        if (sign != lastPowerSign) {
            // Just turned on or changed direction
            lastPowerSign = sign;
            highSince = -1;
            reverseUntil = -1;
            ignoreUntil = now + START_IGNORE_TIME;
        }
        if (reverseUntil >= 0) {
            if (now < reverseUntil) {
                return -sign * REVERSE_POWER;
            }
            reverseUntil = -1;
            lastReverseEnd = now;
            ignoreUntil = now + START_IGNORE_TIME;
        }
        if (gaveUp) {
            return power;
        }
        if (now >= nextCheck) {
            nextCheck = now + CHECK_EVERY;
            lastCurrent = motor.getCurrent(CurrentUnit.AMPS);
        }
        if (now < ignoreUntil || lastCurrent < JAM_CURRENT_AMPS) {
            highSince = -1;
            return power;
        }
        if (highSince < 0) {
            highSince = now;
        }
        if (now - highSince < JAM_TIME) {
            return power;
        }
        // Jammed
        highSince = -1;
        jams++;
        reversalsInARow = now - lastReverseEnd <= REPEAT_WINDOW ? reversalsInARow + 1 : 1;
        if (reversalsInARow > MAX_REVERSALS) {
            gaveUp = true;
            return power;
        }
        reverseUntil = now + REVERSE_TIME;
        return -sign * REVERSE_POWER;
    }

    // Last current read, amps (only read while the intake runs)
    public double getCurrent() {
        return lastCurrent;
    }

    // Jams cleared since the op mode started
    public int getJams() {
        return jams;
    }

    public boolean isReversing() {
        return reverseUntil >= 0;
    }

    // Stopped reversing because it kept jamming (probably full), until the intake is turned off
    public boolean gaveUp() {
        return gaveUp;
    }
}
