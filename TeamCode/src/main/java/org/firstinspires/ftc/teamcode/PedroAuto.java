package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Pedro Auto Red")
public class PedroAuto extends OpMode {
    final double pushServoDown = 0.88;
    final double pushServoUp = 0.3;
    final double blockServoDown = 0.83;
    final double blockServoUp = 0.3;
    private final Pose startPose = new Pose(122.3, 122.3, Math.toRadians(40)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(89, 89, Math.toRadians(45));
    private final Pose turnPose = new Pose(84.1, 82, Math.toRadians(0));// Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1Pose = new Pose(130, 81, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(96, 61, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Pose = new Pose(128, 61, Math.toRadians(0));
    private final Pose park = new Pose(113,74, Math.toRadians(0));

    private DcMotorEx shootMotor = null;
    private Servo pushServo = null;
    private Servo blockServo = null;
    private DcMotorEx intakeMotor = null;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private PathChain scorePreload, runPickup, grabPickup1, scorePickup1, grabPickup2, scorePickup2, backUp, grabPickup3, scorePickup3, turnPath, parkRun;

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
        turnPath = follower.pathBuilder()
                .addPath(new BezierPoint(turnPose))
                .setConstantHeadingInterpolation(turnPose.getHeading())
                .build();
        grabPickup1 = follower.pathBuilder()
                //turnPose
                .addPath(new BezierLine(scorePose, pickup1Pose))
                //ConstantHeading
                .setConstantHeadingInterpolation(pickup1Pose.getHeading())
                .build();
        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), Math.toRadians(40))
                .build();
        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        runPickup = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, pickup3Pose))
                .setConstantHeadingInterpolation(pickup2Pose.getHeading())
                .build();
        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();
        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();
        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        backUp = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, pickup2Pose))
                .setConstantHeadingInterpolation(pickup2Pose.getHeading())
                .build();
        parkRun = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, park))
                .setLinearHeadingInterpolation(scorePose.getHeading(), park.getHeading())
                .build();


    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                blockServo.setPosition(blockServoUp);
                shootMotor.setVelocity(1100);
                follower.followPath(scorePreload, true);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    while (pathTimer.getElapsedTimeSeconds() < 1) {}
                    intakeMotor.setPower(-1);
                    blockServo.setPosition(0.3);
                    for (int x = 0; x < 3; x++) {
                        pushServo.setPosition(pushServoUp);
                        pathTimer.resetTimer();
                        while (pathTimer.getElapsedTimeSeconds() < 0.5) {}
                        pushServo.setPosition(pushServoDown);
                        pathTimer.resetTimer();
                        while (pathTimer.getElapsedTimeSeconds() < 0.5) {}
                    }
                    telemetry.update();
                    pathTimer.resetTimer();
                    while (pathTimer.getElapsedTimeSeconds() < 1) {}
                    blockServo.setPosition(blockServoDown);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup1, 0.6, true);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    intakeMotor.setPower(0);
                    follower.followPath(scorePickup1, 0.8, true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    blockServo.setPosition(blockServoUp);
                    intakeMotor.setPower(-1);
                    for (int x = 0; x < 3; x++) {
                        pushServo.setPosition(pushServoUp);
                        pathTimer.resetTimer();
                        while (pathTimer.getElapsedTimeSeconds() < 0.5) {}
                        pushServo.setPosition(pushServoDown);
                        pathTimer.resetTimer();
                        while (pathTimer.getElapsedTimeSeconds() < 0.5) {}
                    }
                    /* Score Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */

                    follower.followPath(grabPickup2, true);
                    blockServo.setPosition(blockServoDown);
                    setPathState(4);

                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(runPickup);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    while (pathTimer.getElapsedTimeSeconds() < 0.3) {}
                    follower.followPath(backUp);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup2);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    while (pathTimer.getElapsedTimeSeconds() < 1) {}
                    intakeMotor.setPower(-1);
                    blockServo.setPosition(0.3);
                    for (int x = 0; x < 3; x++) {
                        pushServo.setPosition(pushServoUp);
                        pathTimer.resetTimer();
                        while (pathTimer.getElapsedTimeSeconds() < 0.5) {}
                        pushServo.setPosition(pushServoDown);
                        pathTimer.resetTimer();
                        while (pathTimer.getElapsedTimeSeconds() < 0.5) {}
                    }
                    telemetry.update();
                    pathTimer.resetTimer();
                    while (pathTimer.getElapsedTimeSeconds() < 1) {}
                    blockServo.setPosition(blockServoDown);
                    follower.followPath(parkRun,true);
                    setPathState(8);
                }
            case 8:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        shootMotor = hardwareMap.get(DcMotorEx.class, "shootMotor");
        pushServo = hardwareMap.get(Servo.class, "pushServo");
        blockServo = hardwareMap.get(Servo.class, "blockServo");
        shootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pushServo.setPosition(pushServoDown);
        blockServo.setPosition(blockServoDown);

        buildPaths();
        follower.setStartingPose(startPose);
    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }
}
