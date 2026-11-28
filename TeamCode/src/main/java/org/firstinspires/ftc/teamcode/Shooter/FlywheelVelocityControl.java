package org.firstinspires.ftc.teamcode.Shooter;

// Works out the flywheel power for a target RPM, no hardware in here so it can be tested off the robot.
// Feedforward (the volts the target needs) + full power to recover from a shot dip or spin up + a small P and I.
// The hub's own PID overshot 10-21 RPM after every shot (its I term winds up during the dip), the I term here
// stops while far from the target so it can't. Used by CustomFlywheel
public class FlywheelVelocityControl {
    // ---------------- Tuning: feedforward ----------------
    // Volts to hold an RPM = KS + KV * rpm. From FlywheelControlTest 2026-11-27: settled 101-256 RPM fit KV 0.029
    // (motor's theory 0.0276), KS set so 192 RPM matches what the run held (5.8 V). Within 0.15 V at every speed
    public static double KS = 0.30;
    public static double KV = 0.029;
    // Volts per RPM/s of target change, so a moving target (strafing) is followed without lag
    public static double KA = 0.028;
    // The target's change speed is smoothed (fraction of each new reading used) and capped, RPM/s. A 6 RPM jump in one
    // loop used to make almost full power for a moment
    public static double TARGET_ACCEL_BLEND = 0.1;
    public static double MAX_TARGET_ACCEL = 60;

    // ---------------- Tuning: shot recovery / spin up ----------------
    // More than this many RPM under the target = full power until it's back within BOOST_EXIT_RPM
    public static double BOOST_ENTER_RPM = 10;
    // Stops full power this many RPM before the target, the flywheel keeps speeding up for a moment after
    // (reading lag). Raise it if it overshoots after a shot, lower it if it stalls short of the target
    public static double BOOST_EXIT_RPM = 5;
    public static double MAX_POWER = 1.0;
    // Lowest power, 0 = only coasts down
    public static double MIN_POWER = 0.0;
    // Above the target the motor can only coast down (about 20 RPM/s), so this far over it brakes with BRAKE_POWER
    // until it's within BRAKE_EXIT_RPM. 0 power turns braking off. FlywheelTuneTest 2026-11-27: braking at 6 instead
    // of 10 cut the error while the target falls (strafing) from about 5 to 3.2 RPM, holding was no worse.
    // With no brake it never got back down from 250 to 192 in 3 s
    public static double BRAKE_ENTER_RPM = 6;
    public static double BRAKE_EXIT_RPM = 3;
    public static double BRAKE_POWER = -0.15;

    // ---------------- Tuning: feedback ----------------
    // Power per RPM of error
    public static double KP = 0.008;
    // Power per RPM-second of error, only adds up within I_ZONE_RPM of the target and not at full power
    public static double KI = 0.02;
    public static double I_ZONE_RPM = 8;
    public static double MAX_I_POWER = 0.1;
    // ----------------------------------------

    private double integral = 0;
    private boolean boosting = false;
    private boolean braking = false;
    private double lastTarget = Double.NaN;
    private double targetAccel = 0;
    // What the last call did, for logging
    private double lastPower = 0;
    private double lastFeedforward = 0;

    public void reset() {
        integral = 0;
        boosting = false;
        braking = false;
        lastTarget = Double.NaN;
        targetAccel = 0;
    }

    // Power in [MIN_POWER, MAX_POWER] for this loop. dt in seconds, volts = battery voltage
    public double calculate(double targetRPM, double rpm, double dt, double volts) {
        if (targetRPM <= 0) {
            reset();
            lastPower = 0;
            return 0;
        }
        if (!(volts > 6)) {
            volts = 12;
        }
        dt = Math.max(1e-3, Math.min(0.1, dt));
        // Target change speed, smoothed a little since teleop's target moves a bit every loop
        if (!Double.isNaN(lastTarget)) {
            double accel = (targetRPM - lastTarget) / dt;
            accel = Math.max(-MAX_TARGET_ACCEL, Math.min(MAX_TARGET_ACCEL, accel));
            targetAccel += TARGET_ACCEL_BLEND * (accel - targetAccel);
        }
        lastTarget = targetRPM;

        double error = targetRPM - rpm;
        if (error > BOOST_ENTER_RPM) {
            boosting = true;
        } else if (error < BOOST_EXIT_RPM) {
            boosting = false;
        }
        if (BRAKE_POWER < 0 && -error > BRAKE_ENTER_RPM) {
            braking = true;
        } else if (-error < BRAKE_EXIT_RPM) {
            braking = false;
        }

        lastFeedforward = (KS + KV * targetRPM + KA * targetAccel) / volts;
        double power;
        if (boosting) {
            power = MAX_POWER;
        } else if (braking) {
            power = BRAKE_POWER;
        } else {
            if (Math.abs(error) < I_ZONE_RPM) {
                integral += KI * error * dt;
                integral = Math.max(-MAX_I_POWER, Math.min(MAX_I_POWER, integral));
            }
            power = lastFeedforward + KP * error + integral;
        }
        if (!braking) {
            power = Math.max(MIN_POWER, Math.min(MAX_POWER, power));
        }
        lastPower = power;
        return power;
    }

    public boolean isBraking() {
        return braking;
    }

    public boolean isBoosting() {
        return boosting;
    }

    public double getLastPower() {
        return lastPower;
    }

    public double getLastFeedforward() {
        return lastFeedforward;
    }

    public double getIntegral() {
        return integral;
    }
}
