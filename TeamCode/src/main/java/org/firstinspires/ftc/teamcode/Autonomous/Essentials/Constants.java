package org.firstinspires.ftc.teamcode.Autonomous.Essentials;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
        //TODO: how are these parameters derived
        //TODO: who in the Control team is in charge of these
    public static FollowerConstants followerConstants = new FollowerConstants()
            .translationalPIDFCoefficients(new PIDFCoefficients(0.045, 0, 0.01, 0.012))
            .headingPIDFCoefficients(new PIDFCoefficients(0.38, 0, 0.055, 0.055))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.06,0.0,0.02,0.4,0.05))
        //TODO: who in the Control/Mech team has verified the following 3 parameters
            .forwardZeroPowerAcceleration(-25.00)
            .lateralZeroPowerAcceleration(-25.00)
            .centripetalScaling(0.001)
            .mass(9);
    //TODO: as these parameters will likely be different from the two robots our team has, it's better to have at least two different children classes for each of them

    //TODO: when default values are not used, it'll be good to describe the ratoinale briefly, e.g, use 0.9 to apply brake sooner to avoid overshooting
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 0.9, 0.9);


    //This is where the magic happens baby!
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(mecanumConstants)
                .build();
    }

    //TODO: how are the first 3 parameters derived?
    //TODO: who in the Control team is in charge of these
    //TODO: who in the Mech team has verified these, especially the xVelocity and yVelocity
    public static MecanumConstants mecanumConstants = new MecanumConstants()
            .maxPower(.9)
            .xVelocity(65.4)
            .yVelocity(45)
//            .maxPower(0.5)
//            .xVelocity(45)
//            .yVelocity(45)
            .rightFrontMotorName("rightFront")
            .leftFrontMotorName("leftFront")
            .rightRearMotorName("rightBack")
            .leftRearMotorName("leftBack")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);


    // TODO: what platform does this apply to?
    // TODO: who confirmed this in Control team?
    // TODO: who confirmed this in Mech team?
    //This is just instantiating the normal driving
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-60)
            .strafePodX(-155)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

//    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
//            .forwardEncoder_HardwareMapName("rightFront")
//            .forwardEncoderDirection(Encoder.FORWARD)
//            .forwardPodY(-6.1)
//            .strafeEncoder_HardwareMapName("leftFront")
//            .strafeEncoderDirection(Encoder.REVERSE)
//            .strafePodX(3.6)
//            .IMU_HardwareMapName("imu")
//            .IMU_Orientation(
//                    new RevHubOrientationOnRobot(
//                            RevHubOrientationOnRobot.LogoFacingDirection.UP,
//                            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
//                    )
//            );

}
