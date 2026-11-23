package org.firstinspires.ftc.teamcode.Shooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

// Spins the flywheel up with plain power that eases off near the target, then hands over to the hub's velocity PID.
// The hub PID winds up its I term during a long spin up and overshoots, this avoids that. Used by auto and teleop.
public class FlywheelSpinUp {
    // ---------------- Tuning ----------------
    // Fraction of the target RPM where it switches from the power ramp to the hub's RPM control.
    // Higher = the power ramp does more of the spin up. Too high and it can stall just below the switch point.
    public static double SWITCH_FRACTION = 0.92;
    // How hard the ramp pushes per fraction of RPM still missing (power = hold power + RAMP_KP * missing fraction).
    // Higher = faster spin up but more overshoot. Lower = gentler approach to the target.
    public static double RAMP_KP = 3.0;
    // Motor free speed in RPM at full power (435 RPM motor). Used to guess the power that holds the target RPM.
    public static double FREE_SPEED_RPM = 435;
    // Scales that guessed hold power. Raise it if the ramp stalls below SWITCH_FRACTION, lower it if it overshoots.
    public static double HOLD_POWER_SCALE = 1.0;
    // Power limits during the ramp
    public static double RAMP_MAX_POWER = 1.0;
    public static double RAMP_MIN_POWER = 0.0;
    // A new target this many RPM above the old target restarts the ramp (smaller changes stay on RPM control)
    public static double RESTART_RAMP_RPM = 40;
    // ----------------------------------------

    private final DcMotorEx motor;
    private final double ticksPerRev;
    private double targetRPM = 0;
    private boolean ramping = false;
    private DcMotor.RunMode currentMode = null;
    // Last value sent, so the same hub command is not sent every loop
    private double lastPower = Double.NaN;
    private double lastVelocity = Double.NaN;

    public FlywheelSpinUp(DcMotorEx motor, double ticksPerRev) {
        this.motor = motor;
        this.ticksPerRev = ticksPerRev;
    }

    public double getRPM() {
        return motor.getVelocity() / ticksPerRev * 60.0;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public boolean isRamping() {
        return ramping;
    }

    // Call every loop with the wanted RPM
    public void setTargetRPM(double rpm) {
        double current = getRPM();
        if (rpm <= 0) {
            ramping = false;
        } else if ((targetRPM <= 0 || rpm - targetRPM > RESTART_RAMP_RPM) && current < rpm * SWITCH_FRACTION) {
            // Only a new or much higher target ramps, a speed drop from a ball going through stays on RPM control
            ramping = true;
        }
        targetRPM = Math.max(0, rpm);
        update(current);
    }

    private void update(double current) {
        if (targetRPM <= 0) {
            setVelocityMode(0);
            return;
        }
        if (ramping && current >= targetRPM * SWITCH_FRACTION) {
            ramping = false;
        }
        if (ramping) {
            double holdPower = targetRPM / FREE_SPEED_RPM * HOLD_POWER_SCALE;
            double missing = (targetRPM - current) / targetRPM;
            double power = holdPower + RAMP_KP * missing;
            power = Math.max(RAMP_MIN_POWER, Math.min(RAMP_MAX_POWER, power));
            setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if (power != lastPower) {
                motor.setPower(power);
                lastPower = power;
            }
        } else {
            setVelocityMode(targetRPM);
        }
    }

    private void setVelocityMode(double rpm) {
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double ticksPerSecond = rpm / 60.0 * ticksPerRev;
        if (ticksPerSecond != lastVelocity) {
            motor.setVelocity(ticksPerSecond);
            lastVelocity = ticksPerSecond;
        }
    }

    // Switching mode also clears the hub PID, so the RPM control starts without a wound up I term
    private void setMode(DcMotor.RunMode mode) {
        if (mode != currentMode) {
            motor.setMode(mode);
            currentMode = mode;
            lastPower = Double.NaN;
            lastVelocity = Double.NaN;
        }
    }
}
