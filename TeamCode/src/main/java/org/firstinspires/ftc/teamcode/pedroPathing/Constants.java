package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
        .mass(12)
        .forwardZeroPowerAcceleration(-62.38114493205091)
        .lateralZeroPowerAcceleration(-63.11883476320727)
        .useSecondaryTranslationalPIDF(false)
        .useSecondaryHeadingPIDF(false)
        .useSecondaryDrivePIDF(false)
        .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0))
            //P=1
        .headingPIDFCoefficients(new PIDFCoefficients(2, 0, 0.01, 0))
        .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1,0.0,0.0005,0.6,0.0))
        .centripetalScaling(0.0005);


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("right_front_drive")
            .rightRearMotorName("right_back_drive")
            .leftRearMotorName("left_back_drive")
            .leftFrontMotorName("left_front_drive")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(70.86213218164245)
            .yVelocity(57.64534114441837);


    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .forwardTicksToInches(.002974752581)
            .strafeTicksToInches(.00294336865641)
            .turnTicksToInches(.00303500427567)
            .leftPodY(4.1)
            .rightPodY(-4.1)
            .strafePodX(-1.8)
            .leftEncoder_HardwareMapName("left_front_drive")
            .rightEncoder_HardwareMapName("right_back_drive")
            .strafeEncoder_HardwareMapName("left_back_drive")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.FORWARD);



    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .threeWheelLocalizer(localizerConstants)
                .build();
    }
}
