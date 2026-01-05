package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.panels.Panels;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.control.PIDFController;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.JoinedTelemetry;

import java.util.List;
import com.arcrobotics.ftclib.util.InterpLUT;
@Config
@TeleOp(name="NewTeleOp", group="Tuning")
public class NewTeleOp extends LinearOpMode {
    enum AutoState { IDLE, AIMING, SPINNING, FIRING }
    AutoState autoState = AutoState.IDLE;
    private ElapsedTime pushTimer1 = new ElapsedTime();
    private static com.pedropathing.control.PIDFController aimPid;
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
    final double pushServoDown = 0.83; //change if too close to ground: <0.9 == up and >0.9 = down
    final double pushServoUp = 0.3;

    final double blockServoDown = 0.81; //if two balls are shooting at once: <0.81 == up and >0.81 == down
    final double blockServoUp = 0.3;

    public static double Kp = 0.05;
    public static double Kd = 0.0045;
    public static double Kf = 0.025;
    double targetPose = 0;
    public static double rpmTolerance = 150;
    public static double aimTolerance = 2.5;
    double distance;


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
        shootMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        hoodServo.setPosition(hoodPos);
        pushServo.setPosition(pushServoDown);
        blockServo.setPosition(blockServoDown);
        createHoodControlPoints();
        createRPMControlPoints();

        aimPid = new PIDFController(new com.pedropathing.control.PIDFCoefficients(0.05, 0, 0.009, 0));
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
            if (gamepad1.right_trigger > 0.3 && autoState == AutoState.IDLE)
                autoState = AutoState.AIMING;

            if (autoState == AutoState.AIMING) {
                aimPid.updatePosition(getTx());
                aimPid.setTargetPosition(0);
                yaw = -aimPid.run();
                axial = 0; lateral = 0;
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
                case AIMING: if (Math.abs(getTx()) < aimTolerance) autoState = AutoState.SPINNING; break;
                case SPINNING: if (Math.abs(shootMotor.getVelocity() - targetRPM) < rpmTolerance) autoState = AutoState.FIRING; break;
                case FIRING:
                    if (!firingArmed) {
                        firingArmed = true;
                        shots = 3;
                        blockServo.setPosition(blockServoUp);
                    }
                    if (shots == 0) {
                        firingArmed = false;
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
                if (t <= 150) pushServo.setPosition(pushServoUp);
                else if (t <= 300) pushServo.setPosition(pushServoDown);
                else {
                    isPushing = false;
                    shots--;
                    if (shots == 0) blockServo.setPosition(blockServoDown);
                }
            }


            if (gamepad1.rightBumperWasPressed() && !isPushingManual) {
                isPushingManual = true;
                pushTimer.reset();
                pushServo.setPosition(pushServoUp);
                blockServo.setPosition(blockServoUp);
            }
            if (isPushingManual) {
                if (pushTimer.milliseconds() > 400) {
                    pushServo.setPosition(pushServoDown);
                    blockServo.setPosition(blockServoDown);
                    isPushingManual = false;
                }
            }

            telemetry.addData("Aim Ready", aimReady);
            telemetry.addData("Shooter Ready", shooterReady);
            telemetry.addData("Distance (in)", "%.1f", distance);
            telemetry.addData("Target Velocity", targetRPM);
            telemetry.addData("Velocity", "%.0f", shootMotor.getVelocity());
            telemetry.addData("Hood", "%.3f", hoodServo.getPosition());
            telemetry.addData("TX", "%.3f", getTx());
            telemetry.update();


        }
    }


    // ───── Distance Functions ─────
    public double distanceFromTag(double tagID) {
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

    public double distanceFromRed() {
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

    double getTx() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx();
            return tx;
        } else
            return 0;
    }
}
