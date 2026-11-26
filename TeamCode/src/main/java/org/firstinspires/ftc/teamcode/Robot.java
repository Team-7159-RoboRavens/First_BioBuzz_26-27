package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.Autonomous.Essentials.Constants;
import org.firstinspires.ftc.teamcode.ButtonMaps.Mechanumdrive;
import org.firstinspires.ftc.teamcode.Shooter.FlywheelSpinUp;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOpData;

import com.qualcomm.robotcore.hardware.VoltageSensor;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

// Wraps the drivetrain motors, intake motor, and Pinpoint odometry computer,
// exposing simple methods used by the button maps and TeleOp op modes.
public class Robot {
    DcMotorEx leftFront;
    DcMotorEx rightFront;
    DcMotorEx leftBack;
    DcMotorEx rightBack;
    public DcMotorEx shootMotor;
    DcMotorEx intakeMotor1;
    // Reverses the intake for a moment when it stalls, null on robots without the intake motor
    public IntakeJamGuard intakeJamGuard = null;
    public CRServo shootFeedServo;
    public Servo flowerServo;
    public DcMotorEx flywheelMotor;
    public double F = 12;
    public double P = 36;
    public double I = 4;
    public double D = 0;
    GoBildaPinpointDriver pinpoint;

    VoltageSensor voltageSensor;

    InputHandling inputHandler = null;

    CRServo intakeServoLeft;
    CRServo intakeServoRight;


    public List<LynxModule> hubs;

    private final HardwareMap hardwareMap;

    // x and y positions of pinpoint relative to robot center in mm
    //TODO: try to use the values defined from Constants.java, instead of creating two sources of the same values
    private final int startPinpointX = -155;
    private final int startPinpointY = -60;

    public boolean inputHandlingEnabled = false;


    /**
     * @param hmap hardware map used to look up the drivetrain motors, intake motor,
     *             and Pinpoint odometry computer by configuration name
     */
    public Robot(HardwareMap hmap, int robotId) {
        hardwareMap = hmap;
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        if (robotId == 7){
            return;
        }

        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");


        leftFront = hmap.get(DcMotorEx.class, "leftFront");
        rightFront = hmap.get(DcMotorEx.class, "rightFront");
        leftBack = hmap.get(DcMotorEx.class, "leftBack");
        rightBack = hmap.get(DcMotorEx.class, "rightBack");
        shootFeedServo = hmap.get(CRServo.class, "shootFeedServo");

        pinpoint = hmap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Hold position when no power is applied instead of coasting.
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelMotor = hmap.get(DcMotorEx.class,"shootMotor");
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,I,D,F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        // Left front/right back motors are mounted mirrored, so their direction
        // must be reversed for all wheels to drive the same way for a given power sign.
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // Configure the Pinpoint with its physical offset from the robot's center
        // and the odometry pod type/direction, then zero out position and heading.
        if (robotId == 0 || robotId == 4) {
            intakeMotor1 = hmap.get(DcMotorEx.class, "intakeMotor");
            intakeMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        if (robotId == 0 || robotId == 3 || robotId == 4 || robotId == 10) {
            pinpoint = hmap.get(GoBildaPinpointDriver.class, "pinpoint");
            pinpoint.setOffsets(startPinpointX, startPinpointY, DistanceUnit.MM);
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
            if (robotId == 10) {
                // Same pod offsets as Pedro in auto. The old -155 / -60 mm had the forward pod's side flipped,
                // which made the teleop position swing several inches every time the robot turned
                pinpoint.setOffsets(Constants.localizerConfig.xPodOffset.get(),
                        Constants.localizerConfig.yPodOffset.get(), Constants.localizerConfig.offsetUnits.get());
            }
            pinpoint.resetPosAndIMU();
        }

        if (robotId != 3){
            inputHandler = new InputHandling(this);
        }

        if (robotId == 4){
            shootFeedServo = hardwareMap.get(CRServo.class, "shootFeedServo");
            intakeServoRight = hardwareMap.get(CRServo.class, "intakeServoRight");
            intakeServoLeft = hardwareMap.get(CRServo.class, "intakeServoLeft");
            shootMotor = hmap.get(DcMotorEx.class, "shootMotor");
            leftFront.setDirection(DcMotor.Direction.REVERSE);
            rightFront.setDirection(DcMotor.Direction.FORWARD);
            leftBack.setDirection(DcMotor.Direction.FORWARD);
            rightBack.setDirection(DcMotor.Direction.REVERSE);

        }
        if (robotId == 2){
            leftBack.setDirection(DcMotor.Direction.REVERSE);
            rightBack.setDirection(DcMotor.Direction.FORWARD);
        }
        if (robotId == 10){
            shootFeedServo = hardwareMap.get(CRServo.class, "transferServo");
            flowerServo = hardwareMap.get(Servo.class, "flowerServo");
            shootMotor = hmap.get(DcMotorEx.class, "shootMotor");
            intakeMotor1 = hmap.get(DcMotorEx.class, "intakeMotor");
            shootFeedServo.setDirection(CRServo.Direction.REVERSE);
            intakeMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            intakeMotor1.setDirection(DcMotor.Direction.REVERSE);
            intakeJamGuard = new IntakeJamGuard(intakeMotor1);
            leftFront.setDirection(DcMotor.Direction.REVERSE);
            rightFront.setDirection(DcMotor.Direction.FORWARD);
            leftBack.setDirection(DcMotor.Direction.FORWARD);
            rightBack.setDirection(DcMotor.Direction.REVERSE);
            shootMotor.setDirection(DcMotor.Direction.REVERSE);
            shooterSpinUp = new FlywheelSpinUp(shootMotor, SHOOT_TICKS_PER_REV, new PIDFCoefficients(P, I, D, F));
            shooterSpinUp.setBatteryVoltage(voltageSensor.getVoltage());

        }

    }

    public Robot(HardwareMap hmap) {
        this(hmap, 4);
    }

    /**
     * @return hardware map this Robot was constructed with, for classes (e.g. Limelight)
     *         that need to look up their own hardware devices
     */
    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }

    /**
     * @param power intake motor power, [-1, 1]
     */
    public void setIntakePower(double power) {
        if (intakeMotor1 != null) {
            double out = intakeJamGuard != null ? intakeJamGuard.update(power) : power;
            sentIntakePower = sendPower(intakeMotor1, sentIntakePower, out);
        }
        if (intakeServoRight != null && intakeServoLeft != null){
            intakeServoRight.setPower(-power);
            intakeServoLeft.setPower(power);
        }
    }

    // Teleop calls this every loop, so the servo command is only sent when the power changes
    public void setShootFeedPower(double power){
        if (shootFeedServo != null && power != sentShootFeedPower) {
            shootFeedServo.setPower(power);
            sentShootFeedPower = power;
        }
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

    // Power ramp then RPM control (Shooter/FlywheelSpinUp), call every loop
    private FlywheelSpinUp shooterSpinUp = null;
    public void setShootRPMRamped(double RPM){
        if (shooterSpinUp == null) {
            setShootRPM(RPM);
            return;
        }
        shooterSpinUp.setTargetRPM(RPM);
    }

    // Battery reading for the flywheel voltage compensation, call a few times a second
    public void setShooterVoltage(double volts) {
        if (shooterSpinUp != null) {
            shooterSpinUp.setBatteryVoltage(volts);
        }
    }

    // Intake motor speed, ticks per second (bulk cached), 0 without the intake motor
    public double getIntakeVelocity() {
        return intakeMotor1 != null ? intakeMotor1.getVelocity() : 0;
    }

    // True while the spin up is on its power ramp instead of the hub's RPM control
    public boolean isShooterRamping() {
        return shooterSpinUp != null && shooterSpinUp.isRamping();
    }

    // NOMINAL_VOLTAGE / battery, what F and the ramp are scaled by (1 without the spin up)
    public double getShooterVoltageScale() {
        return shooterSpinUp != null ? shooterSpinUp.getVoltageScale() : 1;
    }

    public double getShootRPM(){
        if (shootMotor == null) {
            return 0;
        }
        return shootMotor.getVelocity() / SHOOT_TICKS_PER_REV * 60.0;
    }

    // Runs the powers through InputHandling (heading hold), returns them unchanged when it is off
    public MotorPowers tuneMotorPowers(MotorPowers mp){
        if (inputHandler == null || !inputHandlingEnabled) {
            return mp;
        }
        if (Math.max(Math.max(Math.abs(mp.lf), Math.abs(mp.rf)), Math.max(Math.abs(mp.lb), Math.abs(mp.rb))) < 1e-2) {
            // Sitting still, don't fight pushes, relock once it drives again
            inputHandler.reset();
            return mp;
        }
        return inputHandler.getMotorPowers(mp);
    }

    // lf, rf, lb, rb, same order as MotorPowers and InputHandling
    public DcMotorEx[] getDriveMotors() {
        return new DcMotorEx[]{leftFront, rightFront, leftBack, rightBack};
    }

    public double getBatteryVoltage() {
        return voltageSensor.getVoltage();
    }

    public boolean isHeadingLocked() {
        return inputHandler != null && inputHandler.isLocked();
    }
    /**
     * @param mp per-wheel motor powers to apply to the drivetrain
     */
    public void setPowers(MotorPowers mp) {
        sentDrivePower[0] = sendPower(leftFront, sentDrivePower[0], mp.lf);
        sentDrivePower[1] = sendPower(rightFront, sentDrivePower[1], mp.rf);
        sentDrivePower[2] = sendPower(leftBack, sentDrivePower[2], mp.lb);
        sentDrivePower[3] = sendPower(rightBack, sentDrivePower[3], mp.rb);
    }

    // Each motor power sent is its own hub command, so smaller changes than this are skipped (Pedro does the same).
    // Stopping and changing direction are always sent. 0 sends every change
    public double powerResendThreshold = 0.01;
    // Last power sent to lf, rf, lb, rb, and the intake
    private final double[] sentDrivePower = {Double.NaN, Double.NaN, Double.NaN, Double.NaN};
    private double sentIntakePower = Double.NaN;
    // Last transfer servo power and flower servo position sent, NaN so the first one always goes out
    private double sentShootFeedPower = Double.NaN;
    private double sentFlowerPosition = Double.NaN;

    // Sends the power if it changed enough, returns the power the motor now has
    private double sendPower(DcMotor motor, double sent, double power) {
        boolean changed = Double.isNaN(sent)
                || Math.abs(power - sent) >= powerResendThreshold
                || Math.signum(power) != Math.signum(sent);
        if (!changed) {
            return sent;
        }
        motor.setPower(power);
        return power;
    }

    /**
     * @return current Pinpoint heading, in radians
     */
    public double pinpointAngle() {
        return pinpoint.getHeading(AngleUnit.RADIANS);
    }

    /**
     * @return current Pinpoint X position, in mm
     */
    public double getPinpointX() {
        return pinpoint.getPosX(DistanceUnit.MM);
    }

    /**
     * @return current Pinpoint Y position, in mm
     */
    public double getPinpointY(){
        return pinpoint.getPosY(DistanceUnit.MM);
    }

    // Pinpoint velocity in its own frame, mm/s
    public double getPinpointVelX() {
        return pinpoint.getVelX(DistanceUnit.MM);
    }

    public double getPinpointVelY() {
        return pinpoint.getVelY(DistanceUnit.MM);
    }

    // Pinpoint turn rate, rad/s counterclockwise
    public double getPinpointHeadingVelocity() {
        return pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
    }

    // Must be called once per loop before reading any Pinpoint position/heading data.
    public void pinpointUpdate(){
        pinpoint.update();
    }
    public void resetIMU(){
        pinpoint.resetPosAndIMU();
    }

    public double limelightAngle() { return pinpointAngle();}


    public void setHeadingHold(boolean headingHold){
        if (inputHandler != null) {
            inputHandler.setLockHeading(headingHold);
        }
    }
    // Teleop calls this every loop, so the servo command is only sent when the position changes
    public void setFlowerState(double val) {
        // 0.90 is down, 0.40 is up
        double position = val == 1 ? 0.90 : 0.40;
        if (position != sentFlowerPosition) {
            flowerServo.setPosition(position);
            sentFlowerPosition = position;
        }
    }
}
