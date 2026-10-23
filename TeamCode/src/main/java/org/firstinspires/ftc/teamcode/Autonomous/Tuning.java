package org.firstinspires.ftc.teamcode.Autonomous;

public class Tuning {
    @Tuner
    public static Procedure mecanumTuner() {
        return new org.firstinspires.ftc.teamcode.Autonomous.procedures.MecanumTuner();
    }
}
