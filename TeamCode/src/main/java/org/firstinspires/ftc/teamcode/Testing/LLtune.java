package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.control.PIDFController;
import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.List;


@Config
@Disabled
@TeleOp(name = "LLtune", group = "Tuning")
public class LLtune extends LinearOpMode {
    private enum AutoState { IDLE, AIMING, SPINNING, FIRING }
    private AutoState autoState = AutoState.IDLE;
    private PIDFController pid;
    DcMotor leftFrontDrive = null;
    DcMotor leftBackDrive = null;
    DcMotor rightFrontDrive = null;
    DcMotor rightBackDrive = null;

    private static double tuneKp = 0.025;
    private static double tuneKi = 0.0;
    private static double tuneKd = 0.001;
    private static double tuneKf = 0.15;
    private ElapsedTime pushTimer1 = new ElapsedTime();
    private ElapsedTime fireDelayTimer = new ElapsedTime();
    boolean fireDelayActive = false;

    private DcMotorEx shootMotor, intakeMotor;
    private Servo pushServo = null;
    private Servo blockServo = null;
    private Servo hoodServo = null;
    private Servo light = null;
    private InterpLUT controlPointsRPM = new InterpLUT();
    private InterpLUT controlPointsHood = new InterpLUT();

    Limelight3A limelight;
    ElapsedTime pushTimer = new ElapsedTime();
    boolean isPushing = false;
    boolean isPushingManual = false;
    boolean autoAimActive = false;
    boolean shooterReady = false;
    boolean aimReady = false;

    int intakeOn = 0;
    int shots = 0;
    final double hoodServoClose = 0.48;
    final double pushServoDown = 0.83; //change if too close to ground: <0.9 == up and >0.9 = down
    final double pushServoUp = 0.25;

    final double blockServoDown = 0.81; //if two balls are shooting at once: <0.81 == up and >0.81 == down
    final double blockServoUp = 0.3;
    double targetPose = 0;
    public static double rpmTolerance = 150;
    public static double aimTolerance = 1.5;
    double distance;
    ElapsedTime blockDelayTimer = new ElapsedTime();
    boolean blockDelayActive = false;
    final double BLOCK_OPEN_DELAY = 150;
    public static double targetTx = 0.0;
    public static double maxTurn = 1;


    @Override
    public void runOpMode() {
        boolean locked = false;

        telemetry = new MultipleTelemetry(telemetry, com.acmerobotics.dashboard.FtcDashboard.getInstance().getTelemetry());
        shootMotor = hardwareMap.get(DcMotorEx.class, "shootMotor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        pushServo = hardwareMap.get(Servo.class, "pushServo");
        blockServo = hardwareMap.get(Servo.class, "blockServo");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        light = hardwareMap.get(Servo.class, "light");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        shootMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shootMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new com.qualcomm.robotcore.hardware.PIDFCoefficients(300, 0, 0, 10));
        hoodServo.setPosition(hoodServoClose);
        pushServo.setPosition(pushServoDown);
        blockServo.setPosition(blockServoDown);
        createHoodControlPoints();
        createRPMControlPoints();



        pid = new PIDFController(new PIDFCoefficients(tuneKp,0,tuneKd, tuneKf));
        boolean autoShootActive = false;
        boolean firingArmed = false;
        waitForStart();
        double yaw = 0;

        while (opModeIsActive()) {
            if (gamepad1.circleWasPressed())
                if (intakeOn == 1)
                    intakeOn = 0;
                else
                    intakeOn = 1;
            if (gamepad1.squareWasPressed())
                if (intakeOn == 2)
                    intakeOn = 0;
                else
                    intakeOn = 2;
            if (intakeOn == 1) {
                intakeMotor.setPower(-1);
            } else if (intakeOn == 2) {
                intakeMotor.setPower(1);
            } else {
                intakeMotor.setPower(0);
            }
            double max;
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            yaw = gamepad1.right_stick_x;
            pid.setCoefficients(new PIDFCoefficients(tuneKp,0,tuneKd, tuneKf));

            if (gamepad1.right_trigger > 0.3 && autoState == autoState.IDLE)
                autoState = autoState.AIMING;

            if (autoState == AutoState.AIMING) {
                double error = getTx();
                pid.updateError(error);
                pid.updateFeedForwardInput(Math.signum(error));
                yaw = pid.run();
                if (Math.abs(error) < 2.0) {
                    autoState = autoState.SPINNING;
                    axial = 0;
                    lateral = 0;
                }
            }

/*
            double tx = getTx();
            double error = tx;
            pid.updateError(error);
            pid.updateFeedForwardInput(Math.signum(error));
            double turn = 0;

            if (!locked) {
                yaw = pid.run();
                if (Math.abs(error) < 2.0) {
                    locked = true;
                    axial = 0; lateral = 0;
                }
            } else {
                turn = 0;
            }

 */

            double denominator = Math.max(Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw), 1);
            double leftFrontPower = (axial + lateral + yaw) / denominator;
            double rightFrontPower = (axial - lateral - yaw) / denominator;
            double leftBackPower = (axial - lateral + yaw) / denominator;
            double rightBackPower = (axial + lateral - yaw) / denominator;


            leftFrontDrive.setPower(leftFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightFrontDrive.setPower(rightFrontPower);
            rightBackDrive.setPower(rightBackPower);

            distance = clampDistance(distanceFromRed());

            double targetRPM = controlPointsRPM.get(distance);
           // shootMotor.setVelocity(targetRPM);
            /*
            hoodServo.setPosition(controlPointsHood.get(distance));

            switch (autoState) {
                case SPINNING:
                    if (Math.abs(shootMotor.getVelocity() - targetRPM) < rpmTolerance)
                        autoState = AutoState.FIRING;
                        break;
                case FIRING:
                    if (!fireDelayActive) {
                        fireDelayActive = true;
                        fireDelayTimer.reset();
                        break;
                    }
                    if (fireDelayTimer.seconds() < 0.3) break;

                    // Step 2 – Open blocker and wait before pushing
                    if (!blockDelayActive) {
                        blockServo.setPosition(blockServoUp);
                        blockDelayTimer.reset();
                        blockDelayActive = true;
                        break;
                    }

                    if (blockDelayTimer.milliseconds() < BLOCK_OPEN_DELAY) break;

                    // Step 3 – Arm firing
                    if (!firingArmed) {
                        firingArmed = true;
                        shots = 3;
                    }

                    if (shots == 0) {
                        firingArmed = false;
                        fireDelayActive = false;
                        blockDelayActive = false;
                        autoState = AutoState.IDLE;
                    }
                    break;
            }

            if (shots > 0 && !isPushing) {
                isPushing = true;
                pushTimer1.reset();
                pushServo.setPosition(pushServoUp);
            }

            if (isPushing) {
                double t = pushTimer1.milliseconds();
                if (t <= 150) { //delay time
                    pushServo.setPosition(pushServoUp);
                } else if (t <= 300) {
                    pushServo.setPosition(pushServoDown);
                } else {
                    isPushing = false;
                    shots--;
                    if (shots == 0) {
                        blockServo.setPosition(blockServoDown);
                    }
                }
            }

             */

           telemetry.addData("TX", getTx());
            telemetry.addData("Locked", locked);
           // telemetry.addData("Turn", turn);
          //  telemetry.addData("Error", targetTx - tx);
            telemetry.addData("Kp", pid.P());
            telemetry.addData("KF", pid.F());
            telemetry.addData("Kd", pid.D());
            telemetry.update();
        }
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

    private void createRPMControlPoints() {
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


    private void createHoodControlPoints() {
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

    private double getTx() {
        LLResult r = limelight.getLatestResult();
        return (r != null && r.isValid()) ? r.getTx() : 0;
    }
}