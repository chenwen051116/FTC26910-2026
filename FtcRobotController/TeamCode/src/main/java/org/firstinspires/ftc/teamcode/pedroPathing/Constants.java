package org.firstinspires.ftc.teamcode.pedroPathing;

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

public class    Constants {
    public static DriveEncoderConstants localizerConstants = new DriveEncoderConstants()
            .leftFrontMotorName("frontLeft")
            .leftRearMotorName("backLeft")
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftFrontEncoderDirection(Encoder.FORWARD)
            .leftRearEncoderDirection(Encoder.FORWARD)
            .rightFrontEncoderDirection(Encoder.FORWARD)
            .robotLength(12.1653543)
            .robotWidth(14.4448819)
            .rightRearEncoderDirection(Encoder.FORWARD);

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.337)
//            .forwardZeroPowerAcceleration(-26.465452204903155)
//            .lateralZeroPowerAcceleration(-67.6584184654734)
//            .useSecondaryTranslationalPIDF(true)
//            .useSecondaryHeadingPIDF(true)
//            .useSecondaryDrivePIDF(true)
//            .centripetalScaling(0.00058)
//            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0.00008, 0.025, 0))
//            .headingPIDFCoefficients(new PIDFCoefficients(0.9, 0.0001, 0.1, 0.09))
//            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.01, 0, 0.0005, 0.6, 0.1))
//            .secondaryTranslationalPIDFCoefficients(
//                    new PIDFCoefficients(0.1, 0.0001, 0.012, 0   )
//            )
//            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(2, 0, 0.1, 0))
//            .secondaryDrivePIDFCoefficients(
//                    new FilteredPIDFCoefficients(0.02, 0, 0, 0.6, 0)
//            )
            ;

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(0.8)
            .leftFrontMotorName("frontLeft")
            .leftRearMotorName("backLeft")
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .useBrakeModeInTeleOp(true)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .nominalVoltage(13.2)
            .useVoltageCompensation(true);


    public static PathConstraints pathConstraints = new PathConstraints(
            0.997,
            50,
            1.125,
            1
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }

}
