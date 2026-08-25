package org.firstinspires.ftc.teamcode.ButtonMaps;

import org.firstinspires.ftc.teamcode.MotorPowers;

public class Mechanumdrive {
    public static MotorPowers setMotorPowers(double y, double x, double clockWise){
        return fieldOrientedDrive(y, x, clockWise, Math.PI/2);
    }
    public static MotorPowers fieldOrientedDrive(double forward, double right, double r, double angle){
        double y = Math.sin(angle) * forward + Math.cos(angle) * right;
        double x = Math.sin(angle) * right - Math.cos(angle) * forward;
        MotorPowers motorPower = new MotorPowers();
        motorPower.lf = y + x + r;
        motorPower.rf = y - x - r;
        motorPower.lb = y - x + r;
        motorPower.rb = y + x - r;
        return motorPower;
    }
}
