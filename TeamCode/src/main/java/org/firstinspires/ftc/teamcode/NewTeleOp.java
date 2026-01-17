package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

@Config
@TeleOp(name="NewTeleOp", group="Tuning")
public class NewTeleOp extends LinearOpMode {

    private enum AutoState { IDLE, AIMING, SPINNING, FIRING }
    private com.arcrobotics.ftclib.controller.PIDFController aimPid;
    private AutoState autoState = AutoState.IDLE;
    private ElapsedTime pushTimer1 = new ElapsedTime();
    ElapsedTime fireDelayTimer = new ElapsedTime();
    boolean fireDelayActive = false;

    DcMotorEx shootMotor, intakeMotor;
    Servo hoodServo, pushServo, blockServo, light;
    InterpLUT controlPointsRPM = new InterpLUT();
    InterpLUT controlPointsHood = new InterpLUT();
    DcMotor leftFrontDrive = null;
    DcMotor leftBackDrive = null;
    DcMotor rightFrontDrive = null;
    DcMotor rightBackDrive = null;


    double hoodPos = 0.40;

    Limelight3A limelight;
    ElapsedTime pushTimer = new ElapsedTime();
    boolean isPushing = false;
    boolean isPushingManual = false;
    boolean autoAimActive = false;
    boolean shooterReady = false;
    boolean aimReady = false;

    int intakeOn = 0;
    int shots = 0;
    final double pushServoDown = 0.9; //change if too close to ground: <0.9 == up and >0.9 = down
    final double pushServoUp = 0.5;

    final double blockServoDown = 0.84; //if two balls are shooting at once: <0.81 == up and >0.81 == down
    final double blockServoUp = 0.25;

    public static double Kp = 0.025;
    public static double Ki = 0.0;
    public static double Kd = 0.003;
    public static double Kf = 0.15;
    double targetPose = 0;
    public static double rpmTolerance = 150;
    public static double aimTolerance = 1.5;
    double distance;
    ElapsedTime blockDelayTimer = new ElapsedTime();
    boolean blockDelayActive = false;
    final double BLOCK_OPEN_DELAY = 25;


    @Override
    public void runOpMode() {

        shootMotor = hardwareMap.get(DcMotorEx.class, "shootMotor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        pushServo = hardwareMap.get(Servo.class, "pushServo");
        blockServo = hardwareMap.get(Servo.class, "blockServo");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
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

        limelight.start();
        limelight.pipelineSwitch(0);

        shootMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shootMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new com.qualcomm.robotcore.hardware.PIDFCoefficients(300, 0, 0, 10));
        hoodServo.setPosition(hoodPos);
        pushServo.setPosition(pushServoDown);
        blockServo.setPosition(blockServoDown);
        createHoodControlPoints();
        createRPMControlPoints();

        aimPid = new PIDFController(Kp, 0,Kd,0);
        aimPid.setTolerance(1.5);
        aimPid.setSetPoint(0);
        waitForStart();
        boolean autoShootActive = false;
        boolean firingArmed = false;

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
            //aimPid.setCoefficients(new PIDFCoefficients(0.025,0,0.001, 0.15));

            if (gamepad1.right_trigger > 0.3 && autoState == AutoState.IDLE)
                autoState = AutoState.AIMING;

            if (autoState == AutoState.AIMING) {
                double error = getTx();

                //aimPid.updateFeedForwardInput(Math.signum(error));

                yaw = (-aimPid.calculate(error) + (Kf * Math.signum(error)));;
                if (aimPid.atSetPoint()) {
                    autoState = AutoState.SPINNING;
                    axial = 0;
                    lateral = 0;
                }
            }

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

            shootMotor.setVelocity(targetRPM);
            hoodServo.setPosition(controlPointsHood.get(distance));

            switch (autoState) {
                case SPINNING:
                    if (Math.abs(shootMotor.getVelocity() - targetRPM) < rpmTolerance)
                        autoState = AutoState.FIRING;
                    break;
                case FIRING:
                    /*
                    if (!fireDelayActive) {
                        fireDelayActive = true;
                        fireDelayTimer.reset();
                        break;
                    }
                    if (fireDelayTimer.seconds() < 0.3) break;

                     */

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
                        //fireDelayActive = false;
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





            telemetry.addData("Kp", aimPid.getP());
            telemetry.addData("KF", aimPid.getF());
            telemetry.addData("Kd", aimPid.getD());
            telemetry.addData("State", autoState);
            telemetry.addData("Yaw", yaw);
            telemetry.addData("Distance (in)", "%.1f", distance);
           // telemetry.addData("Target Velocity", targetRPM);
            telemetry.addData("Velocity", "%.0f", shootMotor.getVelocity());
            telemetry.addData("Hood", "%.3f", hoodServo.getPosition());
            telemetry.addData("TX", "%.3f", getTx());
            telemetry.update();
        }
    }


    // ───── Distance Functions ─────
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
        if (distance < 22) return 23;

        // Normal range
        if (distance <= 80) return distance;

        // Hold at 80 until we reach far range
        if (distance < 110) return 79;

        // Far range
        if (distance <= 135) return distance;

        // Clamp anything beyond far range
        return 134;
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
        controlPointsRPM.add(110, 1590);
        controlPointsRPM.add(115, 1590);
        controlPointsRPM.add(120, 1630);
        controlPointsRPM.add(125, 1650);
        controlPointsRPM.add(130, 1680);
        controlPointsRPM.add(135, 1740);

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
        controlPointsHood.add(110, 0.520);
        controlPointsHood.add(115, 0.520);
        controlPointsHood.add(120, 0.524);
        controlPointsHood.add(125, 0.524);
        controlPointsHood.add(130, 0.524);
        controlPointsHood.add(135, 0.526);
        controlPointsHood.createLUT();
    }

    double getTx() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getTx();
        } else
            return 0;
    }
}
