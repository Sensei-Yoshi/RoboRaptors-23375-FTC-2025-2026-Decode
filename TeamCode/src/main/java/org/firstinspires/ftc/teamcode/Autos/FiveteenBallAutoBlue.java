package org.firstinspires.ftc.teamcode.Autos;

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

@Autonomous(name = "15 Ball Blue")
public class FiveteenBallAutoBlue extends OpMode {
    final double pushServoDown = 0.89;
    final double pushServoUp = 0.3;
    final double blockServoDown = 0.84; //if two balls are shooting at once: <0.81 == up and >0.81 == down
    final double blockServoUp = 0.25;
    final double hoodServoClose = 0.48;
    //Change:
    private final Pose startPose = new Pose(122.3, 122.3, Math.toRadians(40)).mirror();
    private final Pose scorePose = new Pose(103, 103, Math.toRadians(41)).mirror();
    private final Pose pickup1Pose = new Pose(125, 85, Math.toRadians(0)).mirror();
    private final Pose pickup3Pose = new Pose(126, 59, Math.toRadians(0)).mirror();
    private final Pose pickup5Pose = new Pose(126, 35, Math.toRadians(0)).mirror();
    private final Pose park = new Pose(113, 74, Math.toRadians(0)).mirror();
    private final Pose gatePose = new Pose(133, 58.5, Math.toRadians(33)).mirror();
    private final Pose pickGatePose = new Pose(133, 58, Math.toRadians(70)).mirror();

    private DcMotorEx shootMotor = null;
    private DcMotorEx shootMotor2 = null;
    private Servo hoodServo = null;
    private Servo pushServo = null;
    private Servo blockServo = null;
    private DcMotorEx intakeMotor = null;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private PathChain scorePreload, grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, parkRun, gatePush, scoreGate, pickGate;

    public void buildPaths() {
        scorePreload = follower.pathBuilder() //shoot first 3 balls
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        gatePush = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, (new Pose(79, 67)).mirror(), gatePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), gatePose.getHeading())
                .build();

        grabPickup1 = follower.pathBuilder() //get next 3
                //turnPose
                .addPath(new BezierCurve(scorePose, (new Pose(67, 82)).mirror(),  pickup1Pose))
                //ConstantHeading
                .setConstantHeadingInterpolation(pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder() //score 3
                .addPath(new BezierLine(gatePose, scorePose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), scorePose.getHeading())
                /*
                 - <45 towards the right, towards the gate
                    - >45 towards left, away from gate
                 */
                .build();

        grabPickup2 = follower.pathBuilder() //gets next 3
                .addPath(new BezierCurve(scorePose,(new Pose(49, 67)).mirror(), pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();


        scorePickup2 = follower.pathBuilder() //scores the 3
                .addPath(new BezierCurve(pickup3Pose,(new Pose(49, 67)).mirror(), scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                /*
               - <45 towards the right, towards the gate
                  - >45 towards left, away from gate
               */
                .build();
        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, (new Pose(64, 17)).mirror(), (new Pose(98, 37)).mirror(), pickup5Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup5Pose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */

        parkRun = follower.pathBuilder() //park
                .addPath(new BezierLine(scorePose, park))
                .setLinearHeadingInterpolation(scorePose.getHeading(), park.getHeading())
                .build();
        scorePickup3 = follower.pathBuilder() //scores the 3
                .addPath(new BezierLine(pickup5Pose, scorePose))
                .setLinearHeadingInterpolation(pickup5Pose.getHeading(), scorePose.getHeading())
                /*
               - <45 towards the right, towards the gate
                  - >45 towards left, away from gate
               */
                .build();
        scoreGate = follower.pathBuilder()
                .addPath(new BezierCurve(gatePose, (new Pose(79, 67)).mirror(), scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();
        pickGate = follower.pathBuilder()
                .addPath(new BezierLine(gatePose, pickGatePose))
                .setConstantHeadingInterpolation(pickGatePose.getHeading())
                .build();




    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                blockServo.setPosition(blockServoUp);
                shootMotor.setVelocity(1110);
                shootMotor2.setVelocity(1110);// Increase or decrease by 5
                follower.followPath(scorePreload, true);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    while (pathTimer.getElapsedTimeSeconds() < 0.4) {}
                    intakeMotor.setPower(-1);
                    pathTimer.resetTimer();
                    while (pathTimer.getElapsedTimeSeconds() < 0.7) {}
                    blockServo.setPosition(blockServoDown);
                    follower.followPath(grabPickup2, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup2, 0.9, true);
                    intakeMotor.setPower(0);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    blockServo.setPosition(blockServoUp);
                    while (pathTimer.getElapsedTimeSeconds() < 0.15) {}
                    intakeMotor.setPower(-1);
                    pathTimer.resetTimer();
                    while (pathTimer.getElapsedTimeSeconds() < 0.7) {}
                    blockServo.setPosition(blockServoDown);
                    follower.followPath(gatePush,true);
                    setPathState(20);
                }
                break;
            case 20: if (!follower.isBusy()) {
                follower.followPath(pickGate,true);
                setPathState(4);
            }
            case 4:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    while (pathTimer.getElapsedTimeSeconds() < 1.4) {}
                    intakeMotor.setPower(0);
                    follower.followPath(scoreGate);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    blockServo.setPosition(blockServoUp);
                    while (pathTimer.getElapsedTimeSeconds() < 0.15) {}
                    intakeMotor.setPower(-1);
                    pathTimer.resetTimer();
                    while (pathTimer.getElapsedTimeSeconds() < 0.7) {}
                    blockServo.setPosition(blockServoDown);
                    follower.followPath(grabPickup1, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    while (pathTimer.getElapsedTimeSeconds() < 0.15) {
                    }
                    intakeMotor.setPower(0);
                    follower.followPath(scorePickup1, 0.9, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    blockServo.setPosition(blockServoUp);
                    while (pathTimer.getElapsedTimeSeconds() < 0.15) {}
                    intakeMotor.setPower(-1);
                    pathTimer.resetTimer();
                    while (pathTimer.getElapsedTimeSeconds() < 0.7) {}
                    blockServo.setPosition(blockServoDown);
                    follower.followPath(grabPickup3, true);
                    setPathState(8);
                }
            case 8:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    while (pathTimer.getElapsedTimeSeconds() < 0.15) {
                    }
                    follower.followPath(scorePickup3,0.9,true);
                    intakeMotor.setPower(0);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    blockServo.setPosition(blockServoUp);
                    while (pathTimer.getElapsedTimeSeconds() < 0.15) {}
                    intakeMotor.setPower(-1);
                    pathTimer.resetTimer();
                    while (pathTimer.getElapsedTimeSeconds() < 0.7) {}
                    blockServo.setPosition(blockServoDown);
                    follower.followPath(parkRun,true);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
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
        shootMotor2 = hardwareMap.get(DcMotorEx.class, "shootMotor2");
        pushServo = hardwareMap.get(Servo.class, "pushServo");
        blockServo = hardwareMap.get(Servo.class, "blockServo");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        shootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        shootMotor2.setDirection(DcMotorEx.Direction.REVERSE);
        shootMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pushServo.setPosition(pushServoDown);
        blockServo.setPosition(blockServoDown);
        hoodServo.setPosition(hoodServoClose);
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
