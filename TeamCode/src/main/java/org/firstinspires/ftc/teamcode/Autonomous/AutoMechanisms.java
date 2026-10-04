package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class AutoMechanisms {
    // I used placeholders cause idk what the names are
    public static final String FLYWHEEL_NAME = "flywheel";
    public static final String INTAKE_SERVO_NAME = "intakeServo";

    // Tune these
    public static final double FLYWHEEL_POWER = 1.0;
    public static final double SERVO_REST_POSITION = 0.0;
    public static final double SERVO_FEED_POSITION = 1.0;
    public static final double SERVO_INTAKE_POSITION = 1.0;

    private final DcMotorEx flywheel;
    private final Servo intakeServo;

    public AutoMechanisms(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, FLYWHEEL_NAME);
        intakeServo = hardwareMap.get(Servo.class, INTAKE_SERVO_NAME);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        reset();
    }

    public void shoot() {
        flywheel.setPower(FLYWHEEL_POWER);
        intakeServo.setPosition(SERVO_FEED_POSITION);
    }

    public void intake() {
        intakeServo.setPosition(SERVO_INTAKE_POSITION);
    }

    public void resetFeeder() {
        intakeServo.setPosition(SERVO_REST_POSITION);
    }

    public void stopFlywheel() {
        flywheel.setPower(0.0);
    }

    public void reset() {
        flywheel.setPower(0.0);
        intakeServo.setPosition(SERVO_REST_POSITION);
    }
}
