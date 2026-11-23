package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Shooter.FlywheelSpinUp;

public class AutoMechanisms {
    public static final String FLYWHEEL_NAME = "shootMotor";
    public static final String INTAKE_MOTOR_NAME = "intakeMotor";
    public static final String SHOOT_FEED_SERVO_NAME = "transferServo";
    public static final String INTAKE_SERVO_RIGHT_NAME = "intakeServoRight";

    // Flywheel encoder ticks per output revolution
    public static final double FLYWHEEL_TICKS_PER_REV = 383.6;
    public static final double FLYWHEEL_P = 12.0;
    public static final double FLYWHEEL_I = 0.5;
    public static final double FLYWHEEL_D = 0.1;
    public static final double FLYWHEEL_F = 14.5;

    public static final double INTAKE_POWER = -0.6;

    // FIX: Changed from 1.0 to -1.0 to spin the continuous servo in the opposite direction
    public static final double SHOOT_FEED_POWER = -1.0;

    private final DcMotorEx flywheel;
    public final DcMotorEx intakeMotor;
    private final CRServo shootFeedServo;
    private final CRServo intakeServoRight;
    // Power ramp then RPM control, so the first shot doesn't overshoot
    private final FlywheelSpinUp flywheelSpinUp;

    public Servo flowerServo;

    public AutoMechanisms(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, FLYWHEEL_NAME);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor = hardwareMap.get(DcMotorEx.class, INTAKE_MOTOR_NAME);
        shootFeedServo = hardwareMap.get(CRServo.class, SHOOT_FEED_SERVO_NAME);
        intakeServoRight = hardwareMap.get(CRServo.class, INTAKE_SERVO_RIGHT_NAME);

        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(FLYWHEEL_P, FLYWHEEL_I, FLYWHEEL_D, FLYWHEEL_F));
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelSpinUp = new FlywheelSpinUp(flywheel, FLYWHEEL_TICKS_PER_REV);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flowerServo = hardwareMap.get(Servo.class, "flowerServo");
        reset();
    }

    // Last commanded state, so the same hub command isn't sent every loop
    private double flywheelRPM = 0;
    private boolean feederOn = true;

    public void spinUpFlywheel(double rpm) {
        setFlywheelRPM(rpm);
    }

    private void setFlywheelRPM(double rpm) {
        flywheelRPM = rpm;
        flywheelSpinUp.setTargetRPM(rpm);
    }

    // Call every loop, the spin up ramp changes its power as the flywheel speeds up
    public void update() {
        flywheelSpinUp.setTargetRPM(flywheelRPM);
    }

    public boolean isFlywheelRamping() {
        return flywheelSpinUp.isRamping();
    }

    public void extendFeeder() {
        if (!feederOn) {
            shootFeedServo.setPower(SHOOT_FEED_POWER);
            feederOn = true;
        }
    }

    public void startShoot(double rpm) {
        stopIntake();
        spinUpFlywheel(rpm);
    }
    public void shoot() {
        extendFeeder();
        intakeMotor.setPower(INTAKE_POWER);
    }

    public void intake() {
        resetFeeder();
        stopFlywheel();
        intakeMotor.setPower(INTAKE_POWER);
    }

    // Intake without stopping the flywheel, for intaking between two volleys
    public void intakeKeepFlywheel() {
        resetFeeder();
        intakeMotor.setPower(INTAKE_POWER);
    }

    public double getFlywheelRPM() {
        return flywheelSpinUp.getRPM();
    }

    public void lowerFlower(){
        flowerServo.setPosition(0.90);
    }

    public void raiseFlower(){
        flowerServo.setPosition(0.4);
    }

    public void resetFeeder() {
        if (feederOn) {
            shootFeedServo.setPower(0.0);
            feederOn = false;
        }
    }

    public void stopIntake() {
        intakeMotor.setPower(0.0);
        intakeServoRight.setPower(0.0);
    }

    public void stopFlywheel() {
        setFlywheelRPM(0.0);
    }

    public void reset() {
        stopFlywheel();
        resetFeeder();
        stopIntake();
    }
}
