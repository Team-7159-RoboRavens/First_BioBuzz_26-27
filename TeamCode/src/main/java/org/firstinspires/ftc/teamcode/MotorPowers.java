package org.firstinspires.ftc.teamcode;

// Simple container for the four mecanum wheel power values.
public class MotorPowers {
    public double lf;
    public double rf;
    public double lb;
    public double rb;
    /**
     * @param lf left front wheel power
     * @param rf right front wheel power
     * @param lb left back wheel power
     * @param rb right back wheel power
     */
    public MotorPowers(double lf, double rf, double lb, double rb) {
        this.lf = lf;
        this.rf = rf;
        this.lb = lb;
        this.rb = rb;
    }

    public MotorPowers() {

    }
}
