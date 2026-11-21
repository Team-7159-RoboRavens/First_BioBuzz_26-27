package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SoloLauncher {
    public DcMotorEx shootMotor;
    public double F = 12;
    public double P = 36;
    public double I = 4;
    public double D = 0;
    private final HardwareMap hardwareMap;

    public SoloLauncher(HardwareMap hmap) {
        hardwareMap = hmap;
        shootMotor = hardwareMap.get(DcMotorEx.class, "shootMotor");
    }

    public void setShootPower(double power){
        if (shootMotor != null) {
            shootMotor.setPower(power);
        }
    }
    // Shoot motor encoder ticks per output revolution, used for both setting and reading RPM
    public static final double SHOOT_TICKS_PER_REV = 383.6;

    public void setShootRPM(double RPM){
        if (shootMotor != null) {
            double targetTicksPerSec = (RPM / 60.0) * SHOOT_TICKS_PER_REV;
            shootMotor.setVelocity(targetTicksPerSec);
        }
    }

    public double getShootRPM(){
        if (shootMotor == null) {
            return 0;
        }
        return shootMotor.getVelocity() / SHOOT_TICKS_PER_REV * 60.0;
    }
}
