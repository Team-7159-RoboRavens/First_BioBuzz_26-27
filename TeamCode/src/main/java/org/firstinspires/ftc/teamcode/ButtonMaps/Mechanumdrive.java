package org.firstinspires.ftc.teamcode.ButtonMaps;

import org.firstinspires.ftc.teamcode.MotorPowers;

// Converts translation/rotation inputs into per-wheel mecanum motor powers.
public class Mechanumdrive {
    /**
     * Robot-centric drive: forward/right/clockwise map directly to wheel powers
     * (angle 90 degrees means "forward" is treated as already field-aligned).
     *
     * @param y forward power, [-1, 1]
     * @param x rightward power, [-1, 1]
     * @param clockWise clockwise rotation power, [-1, 1]
     * @return per-wheel motor powers
     */
    public static MotorPowers setMotorPowers(double y, double x, double clockWise){
        return fieldOrientedDrive(y, x, clockWise, Math.PI/2);
    }

    /**
     * Rotates the forward/right input vector by {@code angle} (field-oriented drive),
     * then scales powers down proportionally if any wheel would exceed 1.
     *
     * @param forward forward power, [-1, 1]
     * @param right rightward power, [-1, 1]
     * @param r clockwise rotation power, [-1, 1]
     * @param angle field-relative angle to rotate the forward/right vector by, in radians
     * @return per-wheel motor powers, scaled down if necessary to stay within [-1, 1]
     */
    public static MotorPowers fieldOrientedDrive(double forward, double right, double r, double angle){
        return fieldOrientedDrive(forward, right, r, angle, new MotorPowers());
    }

    // Same as setMotorPowers, but fills motorPower instead of making a new one
    public static MotorPowers setMotorPowers(double y, double x, double clockWise, MotorPowers motorPower){
        return fieldOrientedDrive(y, x, clockWise, Math.PI/2, motorPower);
    }

    // Same as fieldOrientedDrive, but fills motorPower instead of making a new one
    public static MotorPowers fieldOrientedDrive(double forward, double right, double r, double angle, MotorPowers motorPower){
        double sin = Math.sin(angle);
        double cos = Math.cos(angle);
        double y = sin * forward + cos * right;
        double x = sin * right - cos * forward;
        double max = Math.max(Math.max(Math.abs(y + x + r), Math.abs(y - x - r)), Math.max(Math.abs(y - x + r), Math.abs(y + x - r)));
        if (max > 1){
            y /= max;
            x /= max;
            r /= max;
        }
        // Standard mecanum wheel power mixing formula.
        motorPower.lf = y + x + r;
        motorPower.rf = y - x - r;
        motorPower.lb = y - x + r;
        motorPower.rb = y + x - r;
        return motorPower;
    }
}
