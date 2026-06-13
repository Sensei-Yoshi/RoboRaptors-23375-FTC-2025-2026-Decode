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

@Autonomous(name = "FarShoot Red")
public class FarShootRed extends OpMode {
    final double pushServoDown   = 0.89;
    final double pushServoUp     = 0.3;
    final double blockServoDown  = 0.78;  // matched to AutoAlignTele
    final double blockServoUp    = 0.25;
    final double hoodServoClose  = 0.48;
    final double hoodServoFar    = 0.543; // matched to AutoAlignTele LUT at ~135in

    // ─── PV shooter constants ────────────────────────────────────────────────
    private static final double kS         = 0.09;
    private static final double kV         = 0.0004;
    private static final double kP         = 0.01;
    private static final double TARGET_RPM = 1390; // matched to AutoAlignTele LUT at ~135in
    private static final double RPM_TOL    = 20;

    // ─── Timing constants (seconds) ─────────────────────────────────────────
    private static final double BLOCK_RAISE_DELAY = 0.15;  // delay after raising block before slow intake starts
    private static final double FIRE_DURATION      = 1.2;  // how long intake fires (block stays UP this whole time)
    private static final double FIRE_DURATION_LONG = 1.2;  // longer fire window for shot 4
    private static final double PARK_SETTLE        = 0.5;  // settle time after arriving at park/forward positions

    // ─── Poses ───────────────────────────────────────────────────────────────
    private final Pose startPose     = new Pose(87,    8,    Math.toRadians(90));
    private final Pose scorePose     = new Pose(84.2,  14,   Math.toRadians(67));
    private final Pose park          = new Pose(128,   9, Math.toRadians(0));
    private final Pose runBackPose   = new Pose(123,   9, Math.toRadians(0));
    private final Pose finalparkPose = new Pose(111,   11,   Math.toRadians(0));

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
        return Math.abs(shootMotor.getVelocity()  - TARGET_RPM) < RPM_TOL &&
               Math.abs(shootMotor2.getVelocity() - TARGET_RPM) < RPM_TOL;
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
                .setBrakingStrength(0.88)
                .build();
        runBack = follower.pathBuilder()

                .addPath(new BezierLine(park, runBackPose))
                .setConstantHeadingInterpolation(park.getHeading())
                .setBrakingStrength(0.88)
                .build();
        forwardRun = follower.pathBuilder()

                .addPath(new BezierLine(runBackPose, park))
                .setConstantHeadingInterpolation(park.getHeading())
                .setBrakingStrength(0.88)
                .build();
        score3 = follower.pathBuilder()

                .addPath(new BezierLine(park, scorePose))
                .setLinearHeadingInterpolation(park.getHeading(), scorePose.getHeading())
                .setBrakingStrength(0.88)
                .build();
        finalPark = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, finalparkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), finalparkPose.getHeading())
                .build();
    }

    // =========================================================
    //  State machine
    //
    //  Firing sequence — 3 states per shot:
    //    caseA  – wait path done + atRPMTarget()
    //             → blockServoUp (open gate), reset timer
    //    caseB  – wait BLOCK_RAISE_DELAY
    //             → intakeMotor -0.8 (slow), reset timer
    //             (block stays UP)
    //    caseC  – wait FIRE_DURATION (block UP the entire time)
    //             → blockServoDown (close gate), intakeMotor -1 (full)
    //
    //  Collect cycle — 4 states:
    //    caseD  – followPath(parkRun), reset timer
    //    caseE  – wait path done + PARK_SETTLE → followPath(runBack)
    //    caseF  – wait runBack done → followPath(forwardRun), reset timer
    //    caseG  – wait path done + PARK_SETTLE → followPath(score3)
    //
    //  State map:
    //   0   – start scorePreload
    //   1   – [shot 1] wait path + RPM → raise block
    //   2   – [shot 1] wait BLOCK_RAISE_DELAY → slow intake
    //   3   – [shot 1] wait FIRE_DURATION → close block, full intake
    //   4   – start parkRun (collect 1)
    //   5   – wait done + PARK_SETTLE → runBack
    //   6   – wait runBack done → forwardRun
    //   7   – wait done + PARK_SETTLE → score3
    //   8   – [shot 2] wait path + RPM → raise block
    //   9   – [shot 2] wait BLOCK_RAISE_DELAY → slow intake
    //   10  – [shot 2] wait FIRE_DURATION → close block, full intake
    //   11  – start parkRun (collect 2)
    //   12  – wait done + PARK_SETTLE → runBack
    //   13  – wait runBack done → forwardRun
    //   14  – wait done + PARK_SETTLE → score3
    //   15  – [shot 3] wait path + RPM → raise block
    //   16  – [shot 3] wait BLOCK_RAISE_DELAY → slow intake
    //   17  – [shot 3] wait FIRE_DURATION → close block, full intake
    //   18  – start parkRun (collect 3)
    //   19  – wait done + PARK_SETTLE → runBack
    //   20  – wait runBack done → forwardRun
    //   21  – wait done + PARK_SETTLE → score3
    //   22  – [shot 4] wait path + RPM → raise block
    //   23  – [shot 4] wait BLOCK_RAISE_DELAY → slow intake
    //   24  – [shot 4] wait FIRE_DURATION_LONG → close block, intake off, finalPark
    //   25  – wait finalPark done → -1
    // =========================================================
    public void autonomousPathUpdate() {
        switch (pathState) {

            // ── Drive to first score position ─────────────────────────────────
            case 0:
                blockServo.setPosition(blockServoUp);
                follower.followPath(scorePreload, 0.8, true);
                setPathState(1);
                break;

            // ── Shot 1 ───────────────────────────────────────────────────────
            case 1:
                if (!follower.isBusy() && atRPMTarget()) {
                    blockServo.setPosition(blockServoUp);   // raise gate (open for shooting)
                    setPathState(2);
                }
                break;
            case 2:
                // Gate is UP; wait before starting intake
                if (pathTimer.getElapsedTimeSeconds() > BLOCK_RAISE_DELAY) {
                    intakeMotor.setPower(-0.8);             // slow intake, gate still UP
                    setPathState(3);
                }
                break;
            case 3:
                // Gate stays UP for entire fire window, then close it
                if (pathTimer.getElapsedTimeSeconds() > FIRE_DURATION) {
                    blockServo.setPosition(blockServoDown); // close gate
                    intakeMotor.setPower(-1);               // full intake
                    setPathState(4);
                }
                break;

            // ── Collect cycle 1 ──────────────────────────────────────────────
            case 4:
                follower.followPath(parkRun, 0.8, true);
                setPathState(5);
                break;
            case 5:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > PARK_SETTLE) {
                    follower.followPath(runBack, 0.8, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(forwardRun, 0.8, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > PARK_SETTLE) {
                    follower.followPath(score3, 0.8, true);
                    setPathState(8);
                }
                break;

            // ── Shot 2 ───────────────────────────────────────────────────────
            case 8:
                if (!follower.isBusy() && atRPMTarget()) {
                    blockServo.setPosition(blockServoUp);
                    setPathState(9);
                }
                break;
            case 9:
                if (pathTimer.getElapsedTimeSeconds() > BLOCK_RAISE_DELAY) {
                    intakeMotor.setPower(-0.8);
                    setPathState(10);
                }
                break;
            case 10:
                if (pathTimer.getElapsedTimeSeconds() > FIRE_DURATION) {
                    blockServo.setPosition(blockServoDown);
                    intakeMotor.setPower(-1);
                    setPathState(11);
                }
                break;

            // ── Collect cycle 2 ──────────────────────────────────────────────
            case 11:
                follower.followPath(parkRun, 0.8, true);
                setPathState(12);
                break;
            case 12:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > PARK_SETTLE) {
                    follower.followPath(runBack, 0.8, true);
                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    follower.followPath(forwardRun, 0.8, true);
                    setPathState(14);
                }
                break;
            case 14:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > PARK_SETTLE) {
                    follower.followPath(score3, 0.8, true);
                    setPathState(15);
                }
                break;

            // ── Shot 3 ───────────────────────────────────────────────────────
            case 15:
                if (!follower.isBusy() && atRPMTarget()) {
                    blockServo.setPosition(blockServoUp);
                    setPathState(16);
                }
                break;
            case 16:
                if (pathTimer.getElapsedTimeSeconds() > BLOCK_RAISE_DELAY) {
                    intakeMotor.setPower(-0.8);
                    setPathState(17);
                }
                break;
            case 17:
                if (pathTimer.getElapsedTimeSeconds() > FIRE_DURATION) {
                    blockServo.setPosition(blockServoDown);
                    intakeMotor.setPower(-1);
                    setPathState(18);
                }
                break;

            // ── Collect cycle 3 ──────────────────────────────────────────────
            case 18:
                follower.followPath(parkRun, 0.8, true);
                setPathState(19);
                break;
            case 19:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > PARK_SETTLE) {
                    follower.followPath(runBack, 0.8, true);
                    setPathState(20);
                }
                break;
            case 20:
                if (!follower.isBusy()) {
                    follower.followPath(forwardRun, 0.8, true);
                    setPathState(21);
                }
                break;
            case 21:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > PARK_SETTLE) {
                    follower.followPath(score3, 0.8, true);
                    setPathState(22);
                }
                break;

            // ── Shot 4 ───────────────────────────────────────────────────────
            case 22:
                if (!follower.isBusy() && atRPMTarget()) {
                    blockServo.setPosition(blockServoUp);
                    setPathState(23);
                }
                break;
            case 23:
                if (pathTimer.getElapsedTimeSeconds() > BLOCK_RAISE_DELAY) {
                    intakeMotor.setPower(-0.8);
                    setPathState(24);
                }
                break;
            case 24:
                // Longer fire window for final shot; close gate, intake off, then park
                if (pathTimer.getElapsedTimeSeconds() > FIRE_DURATION_LONG) {
                    blockServo.setPosition(blockServoDown);
                    intakeMotor.setPower(0);
                    follower.followPath(finalPark);
                    setPathState(25);
                }
                break;

            // ── Final park ────────────────────────────────────────────────────
            case 25:
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

        shootMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shootMotor2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shootMotor2.setDirection(DcMotorEx.Direction.REVERSE);

        blockServo.setPosition(blockServoDown);
        hoodServo.setPosition(hoodServoFar);

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