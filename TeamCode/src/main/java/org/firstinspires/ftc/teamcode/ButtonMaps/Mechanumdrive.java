package org.firstinspires.ftc.teamcode.ButtonMaps;

import org.firstinspires.ftc.teamcode.MotorPowers;

public class Mechanumdrive {
    public static MotorPowers setMotorPowers(double y, double x, double clockWise){
        return fieldOrientedDrive(y, x, clockWise, 90);
    }
    public static MotorPowers fieldOrientedDrive(double forward, double right, double r, double angle){
        double y = Math.sin(angle) * forward + Math.cos(angle) * right;
        double x = Math.sin(angle) * right - Math.cos(angle) * forward;
        MotorPowers motorPower = new MotorPowers();
        double max = Math.max(Math.max(Math.abs(y), Math.abs(x)), Math.abs(r));
        if (max > 1){
            y /= max;
            x /= max;
            r /= max;
        }
        motorPower.lf = y + x + r;
        motorPower.rf = y - x - r;
        motorPower.lb = y - x + r;
        motorPower.rb = y + x - r;
        return motorPower;
    }
}
