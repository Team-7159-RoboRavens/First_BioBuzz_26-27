package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

public class Tuning {
    @Tuner
    public static Procedure mecanumTuner() {
        return new org.firstinspires.ftc.teamcode.Autonomous.procedures.MecanumTuner();
    }
}
