package org.firstinspires.ftc.teamcode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Pedro Auto Blue")
public class PedroAutoBlue extends OpMode {
    private DcMotorEx shootMotor = null;
    private Servo pushServo = null;
    private DcMotorEx intakeMotor = null;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(22.3, 122.3, Math.toRadians(130)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(53.6, 90, Math.toRadians(135));
   // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1Pose = new Pose(12, 78.5, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(12,60, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(12,60, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.
   // private final Pose startPoseBlue =
    private final Pose scorePoseBlue = new Pose(53.6, 90, Math.toRadians(135));
    private final Pose pickup1PoseBlue = new Pose(12, 82, Math.toRadians(180));
    private final Pose pickup2PoseBlue = new Pose(12,60, Math.toRadians(0));

    private PathChain scorePreload, grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, turnPath;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */
        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setConstantHeadingInterpolation(pickup1Pose.getHeading())
                .build();
        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), Math.toRadians(130))
                .build();
        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();
        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();
        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();
        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                shootMotor.setVelocity(1100);
                follower.followPath(scorePreload,0.8, true);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    pathTimer.resetTimer();
                    while (pathTimer.getElapsedTimeSeconds() < 1) {}
                    intakeMotor.setPower(-1);
                    for (int x = 0; x<3; x++){
                        pushServo.setPosition(0.3);
                        pathTimer.resetTimer();
                        while (pathTimer.getElapsedTimeSeconds() < 0.5) {}
                        pushServo.setPosition(0.9);
                        pathTimer.resetTimer();
                        while (pathTimer.getElapsedTimeSeconds() < 0.5) {}
                    }
                    telemetry.update();
                    pathTimer.resetTimer();
                    while (pathTimer.getElapsedTimeSeconds() < 1) {}

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup1, 0.3, true);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    intakeMotor.setPower(0);
                    follower.followPath(scorePickup1, true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    intakeMotor.setPower(-1);
                    for (int x = 0; x<3; x++){
                        pushServo.setPosition(0.3);
                        pathTimer.resetTimer();
                        while (pathTimer.getElapsedTimeSeconds() < 0.5) {}
                        pushServo.setPosition(0.9);
                        pathTimer.resetTimer();
                        while (pathTimer.getElapsedTimeSeconds() < 0.5) {}
                    }
                    /* Score Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup2,true);
                    setPathState(4);

                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */

                    setPathState(-1);
                }
                break;
        }
    }
    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
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
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        shootMotor = hardwareMap.get(DcMotorEx.class, "shootMotor");
        pushServo = hardwareMap.get(Servo.class, "pushServo");
        shootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pushServo.setPosition(0.9);
        buildPaths();
        follower.setStartingPose(startPose);
    }
    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}
    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
}

