package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Configurable
public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .useSecondaryTranslationalPIDF(true)
            .mass(11)
            .lateralZeroPowerAcceleration(-73)
            .forwardZeroPowerAcceleration(-43)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.0620, 0, 0, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(0.8, 0, 0.015, 0.04))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025, 0, 0.00001, 0.6, 0.01))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.01, 0, 0.01, 0.6, 0.000005))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1.55, 0, 0.015, 0.04))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.002, 0.05))
            .centripetalScaling(0.000225)
            ;

//    public static DriveEncoderConstants localizerConstants = new DriveEncoderConstants()
//            .rightFrontMotorName("frontRight")
//            .rightRearMotorName("backRight")
//            .leftRearMotorName("backLeft")
//            .leftFrontMotorName("frontLeft")
//            .leftFrontEncoderDirection(Encoder.FORWARD)
//            .leftRearEncoderDirection(Encoder.FORWARD)
//            .rightFrontEncoderDirection(Encoder.FORWARD)
//            .rightRearEncoderDirection(Encoder.FORWARD)
//            ;

    //1: -80.24
    //2: -72.06
    //3: -70.27
    //4: -68.94
    //5: -68.06

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(1)
            .strafePodX(0)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);



    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .xVelocity(78)
            .yVelocity(58)
            .leftFrontMotorName("front_left")
            .rightFrontMotorName("front_right")
            .leftRearMotorName("back_left")
            .rightRearMotorName("back_right")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            ;
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1.25, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                //.driveEncoderLocalizer(localizerConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}