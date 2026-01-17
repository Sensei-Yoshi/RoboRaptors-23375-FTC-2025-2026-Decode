package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
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

@Autonomous(name = "FarShoot Blue")
public class FarShootBlue extends OpMode {
    final double pushServoDown = 0.89;
    final double pushServoUp = 0.3;
    final double blockServoDown = 0.83;
    final double blockServoUp = 0.3;
    final double hoodServoClose = 0.48;
    final double hoodServoFar = 0.52;



    //Change:
    private final Pose startPose = new Pose(87, 8, Math.toRadians(90)).mirror(); // Start Pose of our robot.
    private final Pose scorePose = new Pose(84.2, 9.5, Math.toRadians(60)).mirror(); //100,100
    /*
    - keep x and y same
     -increase or decrease x and y by 2
     - <45 towards the right, towards the gate
     - >45 towards left, away from gate
     */

    /*
    smashing into wall = less x
    not getting all balls = more x
    not aligned with balls = change y
     */

    /*
    pickup2Pose y = pickup3Pose y
    if not aligned = change y

     */
    /*
    if not aligned = change y
    smashing into wall = less x
    not getting all balls = more x
    not aligned with balls = change y
     */

    private final Pose park = new Pose(129,8, Math.toRadians(0)).mirror();

    private DcMotorEx shootMotor = null;
    private Servo hoodServo = null;
    private Servo pushServo = null;
    private Servo blockServo = null;
    private DcMotorEx intakeMotor = null;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private PathChain scorePreload, parkRun;

    public void buildPaths() {
        scorePreload = follower.pathBuilder() //shoot first 3 balls
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
        parkRun = follower.pathBuilder() //shoot first 3 balls
                .addPath(new BezierLine(scorePose, park))
                .setLinearHeadingInterpolation(startPose.getHeading(), park.getHeading())
                .build();



    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                blockServo.setPosition(blockServoUp);
                shootMotor.setVelocity(1580); // Increase or decrease by 5
                follower.followPath(scorePreload,0.6, true);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    while (pathTimer.getElapsedTimeSeconds() < 2.5) {}
                    intakeMotor.setPower(-1);
                    blockServo.setPosition(blockServoUp);
                    while (pathTimer.getElapsedTimeSeconds() < 0.025) {}
                    for (int x = 0; x < 3; x++) {
                        pushServo.setPosition(pushServoUp);
                        pathTimer.resetTimer();
                        while (pathTimer.getElapsedTimeSeconds() < 0.3) {}//delay, 0.1 second increase or decrease {}
                        pushServo.setPosition(pushServoDown);
                        pathTimer.resetTimer();
                        while (pathTimer.getElapsedTimeSeconds() < 0.3) {}
                    }
                    telemetry.update();
                    pathTimer.resetTimer();
                    while (pathTimer.getElapsedTimeSeconds() < 1) {}
                    blockServo.setPosition(blockServoDown);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */

                    follower.followPath(parkRun,true);
                    setPathState(3);
                }
                break;
            case 3:
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
        pushServo = hardwareMap.get(Servo.class, "pushServo");
        blockServo = hardwareMap.get(Servo.class, "blockServo");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        shootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pushServo.setPosition(pushServoDown);
        blockServo.setPosition(blockServoDown);
        hoodServo.setPosition(hoodServoFar);
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


