package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.CRServo;

public class AutoMechanisms {
    public static final String FLYWHEEL_NAME = "shootMotor";
    public static final String INTAKE_MOTOR_NAME = "intakeMotor";
    public static final String SHOOT_FEED_SERVO_NAME = "shootFeedServo";
    public static final String INTAKE_SERVO_LEFT_NAME = "intakeServoLeft";
    public static final String INTAKE_SERVO_RIGHT_NAME = "intakeServoRight";

    public static final double FLYWHEEL_VELOCITY = 1550.0;
    public static final double FLYWHEEL_P = 36.0;
    public static final double FLYWHEEL_I = 4.0;
    public static final double FLYWHEEL_D = 0.0;
    public static final double FLYWHEEL_F = 12.0;
    public static final double INTAKE_POWER = 1.0;
    public static final double SHOOT_FEED_POWER = 0.6;

    private final DcMotorEx flywheel;
    private final DcMotor intakeMotor;
    private final CRServo shootFeedServo;
    private final CRServo intakeServoLeft;
    private final CRServo intakeServoRight;

    public AutoMechanisms(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, FLYWHEEL_NAME);
        intakeMotor = hardwareMap.get(DcMotor.class, INTAKE_MOTOR_NAME);
        shootFeedServo = hardwareMap.get(CRServo.class, SHOOT_FEED_SERVO_NAME);
        intakeServoLeft = hardwareMap.get(CRServo.class, INTAKE_SERVO_LEFT_NAME);
        intakeServoRight = hardwareMap.get(CRServo.class, INTAKE_SERVO_RIGHT_NAME);

        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(FLYWHEEL_P, FLYWHEEL_I, FLYWHEEL_D, FLYWHEEL_F));
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        reset();
    }

    public void shoot() {
        stopIntake();
        flywheel.setVelocity(FLYWHEEL_VELOCITY);
        shootFeedServo.setPower(SHOOT_FEED_POWER);
    }

    public void intake() {
        resetFeeder();
        intakeMotor.setPower(INTAKE_POWER);
        intakeServoLeft.setPower(INTAKE_POWER);
        intakeServoRight.setPower(-INTAKE_POWER);
    }

    public void resetFeeder() {
        shootFeedServo.setPower(0.0);
    }

    public void stopIntake() {
        intakeMotor.setPower(0.0);
        intakeServoLeft.setPower(0.0);
        intakeServoRight.setPower(0.0);
    }

    public void stopFlywheel() {
        flywheel.setVelocity(0.0);
    }

    public void reset() {
        stopFlywheel();
        resetFeeder();
        stopIntake();
    }
}
