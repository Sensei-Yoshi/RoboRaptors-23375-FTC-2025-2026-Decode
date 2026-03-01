
package org.firstinspires.ftc.teamcode.Autos.LimeLight;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Autonomous(name = "15 Ball Blue Limelight")
public class FiveteenBallBlueLL extends OpMode {
    private InterpLUT controlPointsRPM = new InterpLUT();
    private InterpLUT controlPointsHood = new InterpLUT();
    final double pushServoDown = 0.9;
    final double pushServoUp = 0.3;
    final double blockServoDown = 0.84;
    final double blockServoUp = 0.25;
    final double hoodServoClose = 0.48;
    private PIDFController aimPid;

    private static final double AIM_KP = 0.027;
    private static final double AIM_KD = 0.003;
    private static final double AIM_KF = 0.15;
    private static final double AIM_TOLERANCE = 1.7;
    private static final double RPM_TOLERANCE = 150;
    private static final double MAX_ALIGN_TIME = 3.0;

    private final Pose startPose = new Pose(122.3, 122.3, Math.toRadians(40)).mirror();
    private final Pose scorePose = new Pose(97, 97, Math.toRadians(45)).mirror();
    private final Pose turnPose = new Pose(84.1, 82, Math.toRadians(0)).mirror();
    private final Pose pickup1Pose = new Pose(125, 85, Math.toRadians(0)).mirror();
    private final Pose pickup2Pose = new Pose(94, 62, Math.toRadians(0)).mirror();
    private final Pose pickup3Pose = new Pose(126, 61, Math.toRadians(0)).mirror();
    private final Pose pickup4Pose = new Pose(94, 40, Math.toRadians(0)).mirror();
    private final Pose pickup5Pose = new Pose(128, 40, Math.toRadians(0)).mirror();
    private final Pose park = new Pose(113, 74, Math.toRadians(0)).mirror();
    private final Pose gatePose = new Pose(133, 61, Math.toRadians(30)).mirror();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx shootMotor = null;
    private Servo hoodServo = null;
    private Servo pushServo = null;
    private Servo blockServo = null;
    private Limelight3A limelight;
    private Servo light = null;
    private DcMotorEx intakeMotor = null;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer, alignTimer;
    private int pathState;
    private PathChain scorePreload, runPickup, grabPickup1, scorePickup1, grabPickup2, scorePickup2, backUp, grabPickup3, scorePickup3, turnPath, parkRun, gatePush, runPickup2, scoreGate;

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
        turnPath = follower.pathBuilder()
                .addPath(new BezierPoint(turnPose))
                .setConstantHeadingInterpolation(turnPose.getHeading())
                .build();
        gatePush = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, (new Pose(76, 67)).mirror(), gatePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), gatePose.getHeading())
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, (new Pose(67, 82)).mirror(), pickup1Pose))
                .setConstantHeadingInterpolation(pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(gatePose, scorePose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), Math.toRadians(140))
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, (new Pose(49, 67)).mirror(), pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        runPickup = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, pickup3Pose))
                .setConstantHeadingInterpolation(pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup3Pose, (new Pose(49, 67)).mirror(), scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), Math.toRadians(140))
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, (new Pose(65, 29)).mirror(), pickup5Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup4Pose.getHeading())
                .build();

        runPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup4Pose, pickup5Pose))
                .setConstantHeadingInterpolation(pickup4Pose.getHeading())
                .build();

        backUp = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, pickup2Pose))
                .setConstantHeadingInterpolation(pickup2Pose.getHeading())
                .build();

        parkRun = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, park))
                .setLinearHeadingInterpolation(scorePose.getHeading(), park.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup5Pose, scorePose))
                .setLinearHeadingInterpolation(pickup5Pose.getHeading(), Math.toRadians(137))
                .build();

        scoreGate = follower.pathBuilder()
                .addPath(new BezierCurve(gatePose, (new Pose(76, 67)).mirror(), scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), Math.toRadians(137))
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                spinUpShooter();
                follower.followPath(scorePreload, true);
                setPathState(1);
                break;

            case 1:
                // SHOOTING CYCLE 1 - Arrive at score position
                if (!follower.isBusy()) {
                    if (AprilTagfound(20)) {
                        alignTimer.resetTimer();
                        setPathState(2);
                    } else {
                        //spinUpShooter();
                        setPathState(3);
                    }
                }
                break;

            case 2:
                // SHOOTING CYCLE 1 - Aim only
                boolean aimAligned = autoAim();
                //spinUpShooter();

                if (aimAligned) {
                    stopDriveMotors();
                    setPathState(3);
                } else if (alignTimer.getElapsedTimeSeconds() > MAX_ALIGN_TIME) {
                    // stopDriveMotors();
                    setPathState(3);
                }
                break;

            case 3:
                stopDriveMotors();// SHOOTING CYCLE 1 - Wait for shooter, then shoot
                spinUpShooter();
                if (isShooterReady()) {
                    intakeMotor.setPower(-1);
                    blockServo.setPosition(blockServoUp);
                    pathTimer.resetTimer();
                    while (pathTimer.getElapsedTimeSeconds() < 0.029) {}
                    for (int x = 0; x < 3; x++) {
                        pushServo.setPosition(pushServoUp);
                        pathTimer.resetTimer();
                        while (pathTimer.getElapsedTimeSeconds() < 0.15) {}
                        pushServo.setPosition(pushServoDown);
                        pathTimer.resetTimer();
                        while (pathTimer.getElapsedTimeSeconds() < 0.15) {}
                    }
                    blockServo.setPosition(blockServoDown);
                    follower.followPath(grabPickup2, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup2, 0.9, true);
                    setPathState(5);
                }
                break;

            case 5:
                // SHOOTING CYCLE 2 - Arrive at score position
                if (!follower.isBusy()) {
                    if (AprilTagfound(20)) {
                        alignTimer.resetTimer();
                        setPathState(6);
                    } else {
                        // spinUpShooter();
                        setPathState(7);
                    }
                }
                break;

            case 6:
                // SHOOTING CYCLE 2 - Aim only
                boolean aimAligned2 = autoAim();
                // spinUpShooter();

                if (aimAligned2) {
                    telemetry.addLine("Aligned");
                    telemetry.update();
                    stopDriveMotors();
                    setPathState(7);
                } else if (alignTimer.getElapsedTimeSeconds() > MAX_ALIGN_TIME) {
                    telemetry.addLine("Not Aligned");
                    telemetry.update();
                    //  stopDriveMotors();
                    setPathState(7);
                }
                break;

            case 7:
                // SHOOTING CYCLE 2 - Wait for shooter, then shoot
                stopDriveMotors();
                spinUpShooter();
                if (isShooterReady()) {
                    intakeMotor.setPower(-1);
                    blockServo.setPosition(blockServoUp);
                    pathTimer.resetTimer();
                    while (pathTimer.getElapsedTimeSeconds() < 0.025) {}
                    for (int x = 0; x < 3; x++) {
                        pushServo.setPosition(pushServoUp);
                        pathTimer.resetTimer();
                        while (pathTimer.getElapsedTimeSeconds() < 0.15) {}
                        pushServo.setPosition(pushServoDown);
                        pathTimer.resetTimer();
                        while (pathTimer.getElapsedTimeSeconds() < 0.15) {}
                    }
                    blockServo.setPosition(blockServoDown);
                    follower.followPath(gatePush, true);
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    while (pathTimer.getElapsedTimeSeconds() < 1.2) {}
                    intakeMotor.setPower(0);
                    follower.followPath(scoreGate);
                    setPathState(9);
                }
                break;

            case 9:
                // SHOOTING CYCLE 3 - Arrive at score position
                if (!follower.isBusy()) {
                    if (AprilTagfound(20)) {
                        alignTimer.resetTimer();
                        setPathState(10);
                    } else {
                        //spinUpShooter();
                        setPathState(11);
                    }
                }
                break;

            case 10:
                // SHOOTING CYCLE 3 - Aim only
                boolean aimAligned3 = autoAim();
                //spinUpShooter();

                if (aimAligned3) {
                    stopDriveMotors();
                    setPathState(11);
                } else if (alignTimer.getElapsedTimeSeconds() > MAX_ALIGN_TIME) {
                    //stopDriveMotors();
                    setPathState(11);
                }
                break;

            case 11:
                // SHOOTING CYCLE 3 - Wait for shooter, then shoot
                stopDriveMotors();
                spinUpShooter();
                if (isShooterReady()) {
                    intakeMotor.setPower(-1);
                    blockServo.setPosition(blockServoUp);
                    pathTimer.resetTimer();
                    while (pathTimer.getElapsedTimeSeconds() < 0.025) {}
                    for (int x = 0; x < 3; x++) {
                        pushServo.setPosition(pushServoUp);
                        pathTimer.resetTimer();
                        while (pathTimer.getElapsedTimeSeconds() < 0.15) {}
                        pushServo.setPosition(pushServoDown);
                        pathTimer.resetTimer();
                        while (pathTimer.getElapsedTimeSeconds() < 0.15) {}
                    }
                    blockServo.setPosition(blockServoDown);
                    follower.followPath(grabPickup1, true);
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    while (pathTimer.getElapsedTimeSeconds() < 0.06) {}
                    intakeMotor.setPower(0);
                    follower.followPath(scorePickup1, 0.95, true);
                    setPathState(13);
                }
                break;

            case 13:
                // SHOOTING CYCLE 4 - Arrive at score position
                if (!follower.isBusy()) {
                    if (AprilTagfound(20)) {
                        alignTimer.resetTimer();
                        setPathState(14);
                    } else {
                        //spinUpShooter();
                        setPathState(15);
                    }
                }
                break;

            case 14:
                // SHOOTING CYCLE 4 - Aim only
                boolean aimAligned4 = autoAim();
                //spinUpShooter();

                if (aimAligned4) {
                    stopDriveMotors();
                    setPathState(15);
                } else if (alignTimer.getElapsedTimeSeconds() > MAX_ALIGN_TIME) {
                    //  stopDriveMotors();
                    setPathState(15);
                }
                break;

            case 15:
                // SHOOTING CYCLE 4 - Wait for shooter, then shoot
                stopDriveMotors();
                spinUpShooter();
                if (isShooterReady()) {
                    intakeMotor.setPower(-1);
                    blockServo.setPosition(blockServoUp);
                    pathTimer.resetTimer();
                    while (pathTimer.getElapsedTimeSeconds() < 0.025) {}
                    for (int x = 0; x < 3; x++) {
                        pushServo.setPosition(pushServoUp);
                        pathTimer.resetTimer();
                        while (pathTimer.getElapsedTimeSeconds() < 0.15) {}
                        pushServo.setPosition(pushServoDown);
                        pathTimer.resetTimer();
                        while (pathTimer.getElapsedTimeSeconds() < 0.15) {}
                    }
                    blockServo.setPosition(blockServoDown);
                    follower.followPath(grabPickup3, true);
                    setPathState(16);
                }
                break;

            case 16:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    while (pathTimer.getElapsedTimeSeconds() < 0.06) {}
                    follower.followPath(scorePickup3, 0.95, true);
                    setPathState(17);
                }
                break;

            case 17:
                // SHOOTING CYCLE 5 - Arrive at score position
                if (!follower.isBusy()) {
                    if (AprilTagfound(20)) {
                        alignTimer.resetTimer();
                        setPathState(18);
                    } else {
                        //spinUpShooter();
                        setPathState(19);
                    }
                }
                break;

            case 18:
                // SHOOTING CYCLE 5 - Aim only
                boolean aimAligned5 = autoAim();
                //spinUpShooter();

                if (aimAligned5) {
                    telemetry.addLine("Aligned");
                    telemetry.update();
                    stopDriveMotors();
                    setPathState(19);
                } else if (alignTimer.getElapsedTimeSeconds() > MAX_ALIGN_TIME) {
                    telemetry.addLine("Not Aligned");
                    telemetry.update();
                    // stopDriveMotors();
                    setPathState(19);
                }
                break;

            case 19:
                // SHOOTING CYCLE 5 - Wait for shooter, then shoot
                stopDriveMotors();
                spinUpShooter();
                if (isShooterReady()) {
                    intakeMotor.setPower(-1);
                    blockServo.setPosition(blockServoUp);
                    pathTimer.resetTimer();
                    while (pathTimer.getElapsedTimeSeconds() < 0.025) {}
                    for (int x = 0; x < 3; x++) {
                        pushServo.setPosition(pushServoUp);
                        pathTimer.resetTimer();
                        while (pathTimer.getElapsedTimeSeconds() < 0.15) {}
                        pushServo.setPosition(pushServoDown);
                        pathTimer.resetTimer();
                        while (pathTimer.getElapsedTimeSeconds() < 0.15) {}
                    }
                    blockServo.setPosition(blockServoDown);
                    follower.followPath(parkRun, true);
                    setPathState(20);
                }
                break;

            case 20:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    private void stopDriveMotors() {
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("TX:", getTx());
        telemetry.addData("Aim Done:", aimPid.atSetPoint());
        telemetry.addData("Shooter Ready:", isShooterReady());
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        alignTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);

        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shootMotor = hardwareMap.get(DcMotorEx.class, "shootMotor");
        pushServo = hardwareMap.get(Servo.class, "pushServo");
        blockServo = hardwareMap.get(Servo.class, "blockServo");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        light = hardwareMap.get(Servo.class, "light");

        limelight.start();
        limelight.pipelineSwitch(0);

        createRPMControlPoints();
        createHoodControlPoints();

        shootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        aimPid = new PIDFController(AIM_KP, 0, AIM_KD, AIM_KF);
        aimPid.setSetPoint(0);
        aimPid.setTolerance(AIM_TOLERANCE);
        shootMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        pushServo.setPosition(pushServoDown);
        blockServo.setPosition(blockServoDown);
        hoodServo.setPosition(hoodServoClose);

        buildPaths();
        follower.setStartingPose(startPose);
    }

    private boolean AprilTagfound(int tagID) {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return false;

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) return false;

        for (LLResultTypes.FiducialResult t : tags) {
            if (t != null && t.getFiducialId() == tagID) {
                return true;
            }
        }
        return false;
    }

    private double distanceFromTag(double tagID) {
        List<LLResultTypes.FiducialResult> r = limelight.getLatestResult().getFiducialResults();
        if (r.isEmpty()) {
            light.setPosition(0.277);
            return 0.0;
        }

        for (LLResultTypes.FiducialResult i : r) {
            if (i != null && i.getFiducialId() == tagID) {
                light.setPosition(0.500);
                double x = i.getCameraPoseTargetSpace().getPosition().x / DistanceUnit.mPerInch;
                double z = i.getCameraPoseTargetSpace().getPosition().z / DistanceUnit.mPerInch;
                Vector e = new Vector();
                e.setOrthogonalComponents(x, z);
                return e.getMagnitude();
            }
        }
        return 0.0;
    }

    private double clampDistance(double distance) {
        if (distance <= 22) return 23;
        if (distance >= 80) return 79;
        return distance;
    }

    private double distanceFromRed() {
        return distanceFromTag(20);
    }

    public void createRPMControlPoints() {
        controlPointsRPM.add(22, 1100);
        controlPointsRPM.add(25, 1100);
        controlPointsRPM.add(30, 1100);
        controlPointsRPM.add(35, 1100);
        controlPointsRPM.add(40, 1130);
        controlPointsRPM.add(45, 1180);
        controlPointsRPM.add(50, 1200);
        controlPointsRPM.add(55, 1230);
        controlPointsRPM.add(60, 1230);
        controlPointsRPM.add(65, 1230);
        controlPointsRPM.add(70, 1270);
        controlPointsRPM.add(75, 1400);
        controlPointsRPM.add(80, 1490);
        controlPointsRPM.createLUT();
    }

    public void createHoodControlPoints() {
        controlPointsHood.add(22, 0.482);
        controlPointsHood.add(25, 0.482);
        controlPointsHood.add(30, 0.482);
        controlPointsHood.add(35, 0.482);
        controlPointsHood.add(40, 0.482);
        controlPointsHood.add(45, 0.482);
        controlPointsHood.add(50, 0.484);
        controlPointsHood.add(55, 0.490);
        controlPointsHood.add(60, 0.490);
        controlPointsHood.add(65, 0.494);
        controlPointsHood.add(70, 0.498);
        controlPointsHood.add(75, 0.498);
        controlPointsHood.add(80, 0.514);
        controlPointsHood.createLUT();
    }

    private boolean autoAim() {
        if (Math.abs(getTx()) > AIM_TOLERANCE) {
            double error = getTx();

            double yaw = (-aimPid.calculate(error) + (AIM_KF * Math.signum(error)));
            double denominator = Math.max(Math.abs(yaw), 1);
            double leftFrontPower = (yaw) / denominator;
            double rightFrontPower = (-yaw) / denominator;
            double leftBackPower = (yaw) / denominator;
            double rightBackPower = (-yaw) / denominator;

            leftFrontDrive.setPower(leftFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightFrontDrive.setPower(rightFrontPower);
            rightBackDrive.setPower(rightBackPower);

            return aimPid.atSetPoint();
        }
        else return true;
    }

    private void spinUpShooter() {
        double distance = clampDistance(distanceFromRed());
        double targetRPM = controlPointsRPM.get(distance);

        shootMotor.setVelocity(targetRPM);
        hoodServo.setPosition(controlPointsHood.get(distance));
    }

    private boolean isShooterReady() {
        double distance = clampDistance(distanceFromRed());
        double targetRPM = controlPointsRPM.get(distance);
        return (Math.abs(shootMotor.getVelocity() - targetRPM) < RPM_TOLERANCE);
    }

    double getTx() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getTx();
        } else {
            return 0;
        }
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
    }
}
