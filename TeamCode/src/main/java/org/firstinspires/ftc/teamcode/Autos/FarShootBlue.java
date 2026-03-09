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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "FarShoot Blue")
public class FarShootBlue extends OpMode {
    final double pushServoDown  = 0.89;
    final double pushServoUp    = 0.3;
    final double blockServoDown  = 0.78;
    final double blockServoUp    = 0.25;
    final double hoodServoClose = 0.48;
    final double hoodServoFar   = 0.538;

    // ─── PV shooter constants (mirrors TeleOp) ───────────────────────────────
    private static final double kS         = 0.09;
    private static final double kV         = 0.0004;
    private static final double kP         = 0.01;
    private static final double TARGET_RPM = 1375;   // original far-shot velocity
    private static final double RPM_TOL    = 50;

    // ─── Poses ───────────────────────────────────────────────────────────────
    private final Pose startPose = new Pose(87,    8,   Math.toRadians(90)).mirror();
    private final Pose scorePose = new Pose(84.2,  10.3, Math.toRadians(63)).mirror();
    private final Pose park      = new Pose(127,   10.3,   Math.toRadians(0)).mirror();
    private final Pose runBackPose     = new Pose(125,   10.3,   Math.toRadians(0)).mirror();
    private final Pose finalparkPose      = new Pose(111,   11,   Math.toRadians(0)).mirror();



    private DcMotorEx shootMotor  = null;
    private DcMotorEx shootMotor2 = null;
    private Servo     hoodServo   = null;
    private Servo     pushServo   = null;
    private Servo     blockServo  = null;
    private DcMotorEx intakeMotor = null;

    private Follower  follower;
    private Timer     pathTimer, opmodeTimer;
    private int       pathState;

    private PathChain scorePreload, parkRun, runBack, forwardRun, score3, finalPark;

    // =========================================================
    //  PV shooter helpers
    // =========================================================

    private void setShooterPV(double targetRPM) {
        if (targetRPM <= 0) {
            shootMotor.setPower(0);
            shootMotor2.setPower(0);
            return;
        }
        double vel   = shootMotor.getVelocity();
        double power = kS + (kV * targetRPM) + (kP * (targetRPM - vel));
        power = Math.max(-1, Math.min(1, power));
        shootMotor.setPower(power);
        shootMotor2.setPower(power);
    }

    private boolean atRPMTarget() {
        return Math.abs(shootMotor.getVelocity() - TARGET_RPM) < RPM_TOL;
    }

    // =========================================================
    //  Path building
    // =========================================================
    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
        parkRun = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, park))
                .setConstantHeadingInterpolation(park.getHeading())
                .build();
        runBack = follower.pathBuilder()
                .addPath(new BezierLine(park, runBackPose))
                .setConstantHeadingInterpolation(park.getHeading())
                .build();
        forwardRun = follower.pathBuilder()
                .addPath(new BezierLine(runBackPose, park))
                .setConstantHeadingInterpolation(park.getHeading())
                .build();
        score3 = follower.pathBuilder()
                .addPath(new BezierLine(park, scorePose))
                .setLinearHeadingInterpolation(park.getHeading(), scorePose.getHeading())
                .build();
        finalPark = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, finalparkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), finalparkPose.getHeading())
                .build();


    }

    // =========================================================
    //  Autonomous path state machine
    // =========================================================
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                blockServo.setPosition(blockServoUp);
                // Shooter running via loop(); just start driving
                follower.followPath(scorePreload, 0.8, true);
                setPathState(1);
                break;

            case 1:
                // Wait for path AND shooter to be up to speed before firing
                if (!follower.isBusy() && atRPMTarget()) {
                    blockServo.setPosition(blockServoUp);
                    while (pathTimer.getElapsedTimeSeconds() < 0.15) {}
                    intakeMotor.setPower(-0.7);
                    pathTimer.resetTimer();
                    while (pathTimer.getElapsedTimeSeconds() < 0.8) {}
                    blockServo.setPosition(blockServoDown);
                    intakeMotor.setPower(-1);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(parkRun);
                    pathTimer.resetTimer();
                    while (pathTimer.getElapsedTimeSeconds() < 0.2) {}
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(runBack);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(forwardRun);
                    pathTimer.resetTimer();
                    while (pathTimer.getElapsedTimeSeconds() < 0.2) {}
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(score3);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy() && atRPMTarget()) {
                    blockServo.setPosition(blockServoUp);
                    while (pathTimer.getElapsedTimeSeconds() < 0.15) {}
                    intakeMotor.setPower(-0.7);
                    pathTimer.resetTimer();
                    while (pathTimer.getElapsedTimeSeconds() < 0.8) {}
                    blockServo.setPosition(blockServoDown);
                    intakeMotor.setPower(-1);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(parkRun);
                    pathTimer.resetTimer();
                    while (pathTimer.getElapsedTimeSeconds() < 0.2) {}
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(runBack);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(forwardRun);
                    pathTimer.resetTimer();
                    while (pathTimer.getElapsedTimeSeconds() < 0.2) {}
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(score3);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy() && atRPMTarget()) {
                    blockServo.setPosition(blockServoUp);
                    while (pathTimer.getElapsedTimeSeconds() < 0.15) {}
                    intakeMotor.setPower(-0.7);
                    pathTimer.resetTimer();
                    while (pathTimer.getElapsedTimeSeconds() < 0.8) {}
                    blockServo.setPosition(blockServoDown);
                    intakeMotor.setPower(-1);
                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    follower.followPath(parkRun);
                    pathTimer.resetTimer();
                    while (pathTimer.getElapsedTimeSeconds() < 0.2) {}
                    setPathState(14);
                }
                break;
            case 14:
                if (!follower.isBusy()) {
                    follower.followPath(runBack);
                    setPathState(15);
                }
                break;
            case 15:
                if (!follower.isBusy()) {
                    follower.followPath(forwardRun);
                    pathTimer.resetTimer();
                    while (pathTimer.getElapsedTimeSeconds() < 0.2) {}
                    setPathState(16);
                }
                break;
            case 16:
                if (!follower.isBusy()) {
                    follower.followPath(score3);
                    setPathState(17);
                }
                break;
            case 17:
                if (!follower.isBusy() && atRPMTarget()) {
                    blockServo.setPosition(blockServoUp);
                    while (pathTimer.getElapsedTimeSeconds() < 0.15) {}
                    intakeMotor.setPower(-0.7);
                    pathTimer.resetTimer();
                    while (pathTimer.getElapsedTimeSeconds() < 1) {}
                    blockServo.setPosition(blockServoDown);
                    intakeMotor.setPower(-1);
                    setPathState(18);
                }
                break;
            case 18:
                if (!follower.isBusy()) {
                    follower.followPath(parkRun);
                    pathTimer.resetTimer();
                    while (pathTimer.getElapsedTimeSeconds() < 0.2) {}
                    setPathState(19);
                }
                break;
            case 19:
                if (!follower.isBusy()) {
                    follower.followPath(runBack);
                    setPathState(20);
                }
                break;
            case 20:
                if (!follower.isBusy()) {
                    follower.followPath(forwardRun);
                    pathTimer.resetTimer();
                    while (pathTimer.getElapsedTimeSeconds() < 0.2) {}
                    setPathState(21);
                }
                break;
            case 21:
                if (!follower.isBusy()) {
                    follower.followPath(score3);
                    setPathState(22);
                }
                break;
            case 22:
                if (!follower.isBusy() && atRPMTarget()) {
                    blockServo.setPosition(blockServoUp);
                    while (pathTimer.getElapsedTimeSeconds() < 0.15) {}
                    intakeMotor.setPower(-0.7);
                    pathTimer.resetTimer();
                    while (pathTimer.getElapsedTimeSeconds() < 0.8) {}
                    blockServo.setPosition(blockServoDown);
                    intakeMotor.setPower(0);
                    follower.followPath(finalPark);
                    setPathState(23);
                }
                break;
            case 23:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;


        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    // =========================================================
    //  OpMode lifecycle
    // =========================================================
    @Override
    public void loop() {
        follower.update();

        // Drive the PV shooter every loop iteration
        setShooterPV(TARGET_RPM);

        autonomousPathUpdate();

        telemetry.addData("path state",  pathState);
        telemetry.addData("x",           follower.getPose().getX());
        telemetry.addData("y",           follower.getPose().getY());
        telemetry.addData("heading",     Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("shooter RPM", shootMotor.getVelocity());
        telemetry.addData("at target",   atRPMTarget());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer   = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        shootMotor  = hardwareMap.get(DcMotorEx.class, "shootMotor");
        shootMotor2 = hardwareMap.get(DcMotorEx.class, "shootMotor2");
        pushServo   = hardwareMap.get(Servo.class, "pushServo");
        blockServo  = hardwareMap.get(Servo.class, "blockServo");
        hoodServo   = hardwareMap.get(Servo.class, "hoodServo");

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Match TeleOp motor configuration
        shootMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shootMotor2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shootMotor2.setDirection(DcMotorEx.Direction.REVERSE);


        blockServo.setPosition(blockServoDown);
        hoodServo.setPosition(hoodServoFar);   // far-shot hood angle preserved

        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {}
}