package org.firstinspires.ftc.teamcode.Shooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

// Runs the flywheel on plain power with FlywheelVelocityControl instead of the hub's velocity PID.
// Call update() every loop with the bulk cache cleared, and setBatteryVoltage() a few times a second
public class CustomFlywheel {
    // Power changes smaller than this aren't sent, each send is a hub command
    public static double RESEND_POWER = 0.005;

    private final DcMotorEx motor;
    private final double ticksPerRev;
    private final FlywheelVelocityControl control = new FlywheelVelocityControl();
    private double volts = 12;
    private double sentPower = Double.NaN;
    private long lastNanos = 0;
    private boolean modeSet = false;
    private double targetRPM = 0;
    private double lastRPM = 0;

    public CustomFlywheel(DcMotorEx motor, double ticksPerRev) {
        this.motor = motor;
        this.ticksPerRev = ticksPerRev;
    }

    public void setBatteryVoltage(double v) {
        if (v > 6) {
            volts = v;
        }
    }

    // Takes the motor over again, call after something else (the hub PID) ran it
    public void reset() {
        control.reset();
        spunUp = false;
        nearSince = -1;
        targetRPM = 0;
        modeSet = false;
        sentPower = Double.NaN;
        lastNanos = 0;
    }

    // A new target this much higher counts as a new spin up
    public static double RESTART_SPIN_UP_RPM = 40;
    // Within this of the target = spun up
    public static double SPUN_UP_RPM = 6;
    public static double SPUN_UP_TIME = 0.3;
    private boolean spunUp = false;
    private long nearSince = -1;

    // True from a new or much higher target until it first gets close, not during a shot dip
    public boolean isSpinningUp() {
        return targetRPM > 0 && !spunUp;
    }

    public void update(double rpm) {
        double newTarget = Math.max(0, rpm);
        if (newTarget <= 0 || newTarget - targetRPM > RESTART_SPIN_UP_RPM) {
            spunUp = false;
            nearSince = -1;
        }
        targetRPM = newTarget;
        if (!modeSet) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            modeSet = true;
        }
        long now = System.nanoTime();
        double dt = lastNanos == 0 ? 0.015 : (now - lastNanos) / 1e9;
        lastNanos = now;
        lastRPM = motor.getVelocity() / ticksPerRev * 60.0;
        // Has to stay close for SPUN_UP_TIME, the first pass through the target can still undershoot after
        if (!spunUp) {
            if (targetRPM > 0 && Math.abs(targetRPM - lastRPM) < SPUN_UP_RPM) {
                if (nearSince < 0) {
                    nearSince = now;
                }
                if ((now - nearSince) / 1e9 >= SPUN_UP_TIME) {
                    spunUp = true;
                }
            } else {
                nearSince = -1;
            }
        }
        double power = control.calculate(targetRPM, lastRPM, dt, volts);
        if (Double.isNaN(sentPower) || Math.abs(power - sentPower) >= RESEND_POWER || (power == 0) != (sentPower == 0)) {
            motor.setPower(power);
            sentPower = power;
        }
    }

    // Next update sends its power even if it didn't change
    public void resend() {
        sentPower = Double.NaN;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getRPM() {
        return lastRPM;
    }

    public FlywheelVelocityControl getControl() {
        return control;
    }
}
