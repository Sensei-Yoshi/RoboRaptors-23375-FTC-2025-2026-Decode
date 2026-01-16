package org.firstinspires.ftc.teamcode.Autos;

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

@Autonomous(name = "9 Ball Red Limelight")
public class NineBallRedLL extends OpMode {
    private  InterpLUT controlPointsRPM = new InterpLUT();
    private InterpLUT controlPointsHood = new InterpLUT();
    final double pushServoDown = 0.9;
    final double pushServoUp = 0.5;
    final double blockServoDown = 0.84;
    final double blockServoUp = 0.25;
    final double hoodServoClose = 0.48;
    private PIDFController aimPid;

    private static final double AIM_KP = 0.03;
    private static final double AIM_KD = 0.003;
    private static final double AIM_KF = 0.15;
    private static final double AIM_TOLERANCE = 1.5;
    private static final double RPM_TOLERANCE = 150;
    private static final double MAX_ALIGN_TIME = 3.0; // Maximum time to spend aligning (seconds)

    //Change:
    private final Pose startPose = new Pose(122.3, 122.3, Math.toRadians(40)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(103, 103, Math.toRadians(45)); //100,100
    private final Pose turnPose = new Pose(84.1, 82, Math.toRadians(0)); //ignore
    private final Pose pickup1Pose = new Pose(128, 83, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(94, 62, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(128, 62, Math.toRadians(0));
    private final Pose park = new Pose(113,74, Math.toRadians(0));

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
    private PathChain scorePreload, runPickup, grabPickup1, scorePickup1, grabPickup2, scorePickup2, backUp, grabPickup3, scorePickup3, turnPath, parkRun;

    public void buildPaths() {
        scorePreload = follower.pathBuilder() //shoot first 3 balls
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
        turnPath = follower.pathBuilder()
                .addPath(new BezierPoint(turnPose))
                .setConstantHeadingInterpolation(turnPose.getHeading())
                .build();
        grabPickup1 = follower.pathBuilder() //get next 3
                .addPath(new BezierCurve(scorePose, (new Pose(67, 82)),  pickup1Pose))
                .setConstantHeadingInterpolation(pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder() //score 3
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), Math.toRadians(40))
                .build();

        grabPickup2 = follower.pathBuilder() //gets next 3
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        runPickup = follower.pathBuilder() //gets same 3 balls
                .addPath(new BezierLine(pickup2Pose, pickup3Pose))
                .setConstantHeadingInterpolation(pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder() //scores the 3
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), Math.toRadians(42))
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        backUp = follower.pathBuilder() //backs up
                .addPath(new BezierLine(pickup3Pose, pickup2Pose))
                .setConstantHeadingInterpolation(pickup2Pose.getHeading())
                .build();
        parkRun = follower.pathBuilder() //park
                .addPath(new BezierLine(scorePose, park))
                .setLinearHeadingInterpolation(scorePose.getHeading(), park.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Move to score position
                follower.followPath(scorePreload, true);
                setPathState(1);
                break;

            case 1:
                // Wait for robot to reach score position
                if (!follower.isBusy()) {
                    // Check if AprilTag is visible
                    if (AprilTagfound(24)) {
                        telemetry.addLine("AprilTag 24 found - starting alignment");
                        alignTimer.resetTimer();
                        setPathState(2); // Move to alignment state
                    } else {
                        telemetry.addLine("WARNING: AprilTag 24 not found - shooting without alignment");
                        setPathState(3); // Skip alignment, go straight to shooting
                    }
                }
                break;

            case 2:
                // Align with AprilTag and spin up shooter
                boolean aligned = autoAimAndSpinUp();




                // Check if aligned or timeout
                if (aligned) {
                    telemetry.addLine("Alignment complete - ready to shoot");
                    stopDriveMotors(); // Stop alignment movement
                    setPathState(3); // Move to shooting state
                } else if (alignTimer.getElapsedTimeSeconds() > MAX_ALIGN_TIME) {
                    telemetry.addLine("Alignment timeout - shooting anyway");
                    stopDriveMotors();
                    setPathState(3);
                }
                break;

            case 3:
                // Shoot the balls
                stopDriveMotors(); // Ensure robot is stopped
                intakeMotor.setPower(-1);
                blockServo.setPosition(blockServoUp);
                pathTimer.resetTimer();
                while (pathTimer.getElapsedTimeSeconds() < 0.1) {}

                for (int x = 0; x < 3; x++) {
                    pushServo.setPosition(pushServoUp);
                    pathTimer.resetTimer();
                    while (pathTimer.getElapsedTimeSeconds() < 0.15) {}
                    pushServo.setPosition(pushServoDown);
                    pathTimer.resetTimer();
                    while (pathTimer.getElapsedTimeSeconds() < 0.15) {}
                }

                pathTimer.resetTimer();
                while (pathTimer.getElapsedTimeSeconds() < 1) {}
                blockServo.setPosition(blockServoDown);

                follower.followPath(grabPickup1, 0.6, true);
                setPathState(4);
                break;

            case 4:
                // Pick up first set of balls
                if (!follower.isBusy()) {
                    intakeMotor.setPower(0);
                    follower.followPath(scorePickup1, 0.8, true);
                    setPathState(5);
                }
                break;

            case 5:
                // Return to score position for second shooting
                if (!follower.isBusy()) {
                    if (AprilTagfound(24)) {
                        alignTimer.resetTimer();
                        setPathState(6); // Align again
                    } else {
                        setPathState(7); // Skip alignment
                    }
                }
                break;

            case 6:
                // Align for second shooting
                boolean aligned2 = autoAimAndSpinUp();

                if (aligned2 || alignTimer.getElapsedTimeSeconds() > MAX_ALIGN_TIME) {
                    stopDriveMotors();
                    setPathState(7);
                }
                break;

            case 7:
                // Shoot second set of balls
                stopDriveMotors();
                blockServo.setPosition(blockServoUp);
                pathTimer.resetTimer();
                while (pathTimer.getElapsedTimeSeconds() < 0.1) {}
                intakeMotor.setPower(-1);

                for (int x = 0; x < 3; x++) {
                    pushServo.setPosition(pushServoUp);
                    pathTimer.resetTimer();
                    while (pathTimer.getElapsedTimeSeconds() < 0.15) {}
                    pushServo.setPosition(pushServoDown);
                    pathTimer.resetTimer();
                    while (pathTimer.getElapsedTimeSeconds() < 0.15) {}
                }

                follower.followPath(grabPickup2, true);
                blockServo.setPosition(blockServoDown);
                setPathState(8);
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(runPickup);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    while (pathTimer.getElapsedTimeSeconds() < 0.3) {}
                    follower.followPath(backUp);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup2, 0.8, true);
                    setPathState(11);
                }
                break;

            case 11:
                // Return to score position for third shooting
                if (!follower.isBusy()) {
                    if (AprilTagfound(24)) {
                        alignTimer.resetTimer();
                        setPathState(12); // Align again
                    } else {
                        setPathState(13); // Skip alignment
                    }
                }
                break;

            case 12:
                // Align for third shooting
                boolean aligned3 = autoAimAndSpinUp();

                if (aligned3 || alignTimer.getElapsedTimeSeconds() > MAX_ALIGN_TIME) {
                    stopDriveMotors();
                    setPathState(13);
                }
                break;

            case 13:
                // Shoot third set of balls
                stopDriveMotors();
                pathTimer.resetTimer();
                while (pathTimer.getElapsedTimeSeconds() < 1) {}
                intakeMotor.setPower(-1);
                blockServo.setPosition(blockServoUp);
                pathTimer.resetTimer();
                while (pathTimer.getElapsedTimeSeconds() < 0.1) {}

                for (int x = 0; x < 3; x++) {
                    pushServo.setPosition(pushServoUp);
                    pathTimer.resetTimer();
                    while (pathTimer.getElapsedTimeSeconds() < 0.15) {}
                    pushServo.setPosition(pushServoDown);
                    pathTimer.resetTimer();
                    while (pathTimer.getElapsedTimeSeconds() < 0.15) {}
                }

                pathTimer.resetTimer();
                while (pathTimer.getElapsedTimeSeconds() < 1) {}
                blockServo.setPosition(blockServoDown);
                follower.followPath(parkRun, true);
                setPathState(14);
                break;

            case 14:
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
        telemetry.addData("Tx Error", getTx());
        telemetry.addData("Shooter RPM", shootMotor.getVelocity());
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
        return distanceFromTag(24);
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

    private boolean autoAimAndSpinUp() {
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

        double distance = clampDistance(distanceFromRed());
        double targetRPM = controlPointsRPM.get(distance);

        shootMotor.setVelocity(targetRPM);
        hoodServo.setPosition(controlPointsHood.get(distance));

        return (aimPid.atSetPoint() && ((Math.abs(shootMotor.getVelocity() - targetRPM) < RPM_TOLERANCE)));
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