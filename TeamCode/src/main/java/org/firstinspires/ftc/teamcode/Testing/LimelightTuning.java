package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

@Config
@TeleOp(name = "LimelightShotTuning", group = "Tuning")
public class LimelightTuning extends LinearOpMode {

    // ─────────────────────────────────────────────
    //  Shoot state machine  (no aiming state)
    // ─────────────────────────────────────────────
    private enum ShootState { IDLE, SPINNING, FIRING }
    private ShootState shootState = ShootState.IDLE;

    // ─────────────────────────────────────────────
    //  Hardware
    // ─────────────────────────────────────────────
    private DcMotorEx   shootMotor, intakeMotor, shootMotor2;
    private Servo       hoodServo, pushServo, blockServo;
    private Limelight3A limelight;

    // ─────────────────────────────────────────────
    //  PV Shooter constants — tune live via Dashboard
    // ─────────────────────────────────────────────
    public static double kS = 0.09;     // static / friction offset
    public static double kV = 0.0004;  // velocity feedforward (power per tick/s)
    public static double kP = 0.01;     // proportional error gain

    // ─────────────────────────────────────────────
    //  Shooter target — adjust with D-pad up/down
    // ─────────────────────────────────────────────
    public static double targetVelocity = 1620;
    public static double hoodPos        = 0.538;
    public static double intakeSpeed        = -0.75;

    public static double rpmTolerance        = 50;
    public static double BLOCK_OPEN_DURATION_MS = 1500;  // total gate-open time
    public static double INTAKE_DELAY_MS        = 200;   // delay before intake starts

    // ─────────────────────────────────────────────
    //  Servo constants
    // ─────────────────────────────────────────────
    private static final double PUSH_SERVO_DOWN  = 0.9;
    private static final double PUSH_SERVO_UP    = 0.5;
    private static final double BLOCK_SERVO_DOWN = 0.78;
    private static final double BLOCK_SERVO_UP   = 0.25;

    // ─────────────────────────────────────────────
    //  State
    // ─────────────────────────────────────────────
    private final ElapsedTime blockTimer = new ElapsedTime();
    private boolean isBlocking          = false;
    private int     intakeOn            = 0;
    private int     preShootIntakeState = 0;

    // =========================================================
    //  runOpMode
    // =========================================================
    @Override
    public void runOpMode() {
        shootMotor  = hardwareMap.get(DcMotorEx.class, "shootMotor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        shootMotor2 = hardwareMap.get(DcMotorEx.class, "shootMotor2");

        hoodServo  = hardwareMap.get(Servo.class, "hoodServo");
        pushServo  = hardwareMap.get(Servo.class, "pushServo");
        blockServo = hardwareMap.get(Servo.class, "blockServo");
        limelight  = hardwareMap.get(Limelight3A.class, "limelight");

        shootMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shootMotor2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shootMotor2.setDirection(DcMotorEx.Direction.REVERSE);

        limelight.start();
        limelight.pipelineSwitch(0);

        hoodServo.setPosition(hoodPos);
        pushServo.setPosition(PUSH_SERVO_DOWN);
        blockServo.setPosition(BLOCK_SERVO_DOWN);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {

            // ── D-pad: adjust target velocity and hood ──
            if (gamepad1.dpadUpWasPressed())    targetVelocity += 10;
            if (gamepad1.dpadDownWasPressed())  targetVelocity -= 10;
            if (gamepad1.dpadRightWasPressed()) hoodPos        += 0.002;
            if (gamepad1.dpadLeftWasPressed())  hoodPos        -= 0.002;
            if (gamepad1.aWasPressed())  intakeSpeed        -= 0.1;
            if (gamepad1.yWasPressed())  intakeSpeed        += 0.1;

            targetVelocity = Math.max(0, targetVelocity);
            hoodPos        = clamp(hoodPos, 0.0, 1.0);

            // ── Shooter always running ──
            setShooterPV(targetVelocity);
            hoodServo.setPosition(hoodPos);

            // ── Trigger: start shoot sequence ──
            if (gamepad1.right_trigger > 0.3 && shootState == ShootState.IDLE) {
                preShootIntakeState = intakeOn;
                intakeOn = 0;
                intakeMotor.setPower(0);
                shootState = ShootState.SPINNING;
            }

            // ── Intake toggle — only when IDLE ──
            if (shootState == ShootState.IDLE) {
                if (gamepad1.circleWasPressed()) intakeOn = (intakeOn == 1) ? 0 : 1;
                if (gamepad1.squareWasPressed()) intakeOn = (intakeOn == 2) ? 0 : 2;
                switch (intakeOn) {
                    case 1:  intakeMotor.setPower(intakeSpeed); break;
                    case 2:  intakeMotor.setPower( 1); break;
                    default: intakeMotor.setPower( 0); break;
                }
            }

            // ── State machine ──
            runStateMachine();

            updateTelemetry();
        }
    }

    // =========================================================
    //  State machine
    // =========================================================
    private void runStateMachine() {
        switch (shootState) {

            case SPINNING:
                // Wait for RPM to reach target
                if (atRPMTarget()) {
                    shootState = ShootState.FIRING;
                }
                break;

            case FIRING:
                // Step 1: open gate once at entry
                if (!isBlocking) {
                    isBlocking = true;
                    blockTimer.reset();
                    blockServo.setPosition(BLOCK_SERVO_UP);
                }

                // Step 2: start intake after servo has had time to travel up
                if (blockTimer.milliseconds() >= INTAKE_DELAY_MS) {
                    intakeMotor.setPower(intakeSpeed);
                }

                // Step 3: close gate, stop intake, restore previous intake state
                if (blockTimer.milliseconds() >= BLOCK_OPEN_DURATION_MS) {
                    blockServo.setPosition(BLOCK_SERVO_DOWN);
                    intakeMotor.setPower(0);
                    intakeOn   = preShootIntakeState;
                    isBlocking = false;
                    shootState = ShootState.IDLE;
                }
                break;

            default:
                break;
        }
    }

    // =========================================================
    //  PV Shooter Controller
    //
    //   power = kS  +  kV × target  +  kP × (target − actual)
    // =========================================================
    private void setShooterPV(double targetRPM) {
        if (targetRPM <= 0) {
            shootMotor.setPower(0);
            shootMotor2.setPower(0);
            return;
        }

        double shootVel   = shootMotor.getVelocity();
        double shootPower = kS + (kV * targetRPM) + (kP * (targetRPM - shootVel));

        shootMotor.setPower(clamp(shootPower, -1, 1));
        shootMotor2.setPower(clamp(shootPower, -1, 1));
    }

    private boolean atRPMTarget() {
        return Math.abs(shootMotor.getVelocity() - targetVelocity) < rpmTolerance;
    }

    // =========================================================
    //  Telemetry
    // =========================================================
    private void updateTelemetry() {
        double shootVel = shootMotor.getVelocity();
        double shootErr = targetVelocity - shootVel;

        telemetry.addData("State/ShootState",         shootState.toString());
        telemetry.addData("State/IsBlocking",         isBlocking);
        telemetry.addData("State/BlockTimerMs",       blockTimer.milliseconds());

        telemetry.addData("Shooter/TargetVelocity",   targetVelocity);
        telemetry.addData("Shooter/ActualVelocity",   shootVel);
        telemetry.addData("Shooter/Error",            shootErr);
        telemetry.addData("Shooter/Power",            shootMotor.getPower());
        telemetry.addData("Shooter/AtTarget",         atRPMTarget());

        telemetry.addData("Hood/Position",            hoodPos);
        telemetry.addData("Distance/in",              "%.1f", distanceFromRed());

        telemetry.addData("Intake/State",             intakeOn);
        telemetry.addData("Intake Speed",             intakeSpeed);
        telemetry.addData("Intake/PreShootState",     preShootIntakeState);

        telemetry.addData("PV/kS",                   kS);
        telemetry.addData("PV/kV",                   kV);
        telemetry.addData("PV/kP",                   kP);
        telemetry.addData("PV/term_kV",              kV * targetVelocity);
        telemetry.addData("PV/term_kP",              kP * shootErr);

        telemetry.update();
    }

    // =========================================================
    //  Distance helpers
    // =========================================================
    private double distanceFromTag(double tagID) {
        if (limelight.getLatestResult() == null) return 0;
        List<LLResultTypes.FiducialResult> results = limelight.getLatestResult().getFiducialResults();
        if (results == null || results.isEmpty()) return 0;

        for (LLResultTypes.FiducialResult f : results) {
            if (f != null && f.getFiducialId() == tagID) {
                double x = f.getCameraPoseTargetSpace().getPosition().x / DistanceUnit.mPerInch;
                double z = f.getCameraPoseTargetSpace().getPosition().z / DistanceUnit.mPerInch;
                Vector v = new Vector();
                v.setOrthogonalComponents(x, z);
                return v.getMagnitude();
            }
        }
        return 0;
    }

    private double distanceFromRed() {
        return distanceFromTag(24);
    }

    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}