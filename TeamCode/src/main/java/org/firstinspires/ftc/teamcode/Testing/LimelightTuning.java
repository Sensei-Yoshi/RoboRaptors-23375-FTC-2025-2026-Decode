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
    //  Hardware
    // ─────────────────────────────────────────────
    private DcMotorEx   shootMotor, intakeMotor, shootMotor2;
    private Servo       hoodServo, pushServo, blockServo;
    private Limelight3A limelight;

    // ─────────────────────────────────────────────
    //  PV Shooter constants — tune live via Dashboard
    //  Kept identical to NewTeleOp for consistency
    // ─────────────────────────────────────────────
    public static double kS      = 0.09;    // static / friction offset
    public static double kV      = 0.00038; // velocity feedforward (power per tick/s)
    public static double kP      = 0.01;   // proportional error gain (both motors)
  // static friction offset  (lift motor — tune separately)

    // ─────────────────────────────────────────────
    //  Shooter target — adjust with D-pad up/down
    // ─────────────────────────────────────────────
    public static double targetVelocity = 1620;
    public static double hoodPos        = 0.538;

    // ─────────────────────────────────────────────
    //  Servo constants
    // ─────────────────────────────────────────────
    private static final double PUSH_SERVO_DOWN  = 0.9;
    private static final double PUSH_SERVO_UP    = 0.5;
    private static final double BLOCK_SERVO_DOWN = 0.78;
    private static final double BLOCK_SERVO_UP   = 0.25;

    // ─────────────────────────────────────────────
    //  Timers / state
    // ─────────────────────────────────────────────
    private final ElapsedTime pushTimer      = new ElapsedTime();
    private final ElapsedTime rapidFireTimer = new ElapsedTime();

    private boolean isPushing       = false;
    private boolean isPushingManual = false;
    private int     shots           = 0;
    private int     intakeOn        = 0;

    // =========================================================
    //  runOpMode
    // =========================================================
    @Override
    public void runOpMode() {
        shootMotor  = hardwareMap.get(DcMotorEx.class, "shootMotor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        shootMotor2 = hardwareMap.get(DcMotorEx.class, "liftMotor");

        hoodServo  = hardwareMap.get(Servo.class, "hoodServo");
        pushServo  = hardwareMap.get(Servo.class, "pushServo");
        blockServo = hardwareMap.get(Servo.class, "blockServo");
        limelight  = hardwareMap.get(Limelight3A.class, "limelight");

        // RUN_WITHOUT_ENCODER: PV controller drives power directly.
        // getVelocity() still works on DcMotorEx regardless of run mode.
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

            targetVelocity = Math.max(0, targetVelocity);
            hoodPos        = clamp(hoodPos, 0.0, 1.0);

            // ── PV controller (matches NewTeleOp setShooterPV exactly) ──
            setShooterPV(targetVelocity);
            hoodServo.setPosition(hoodPos);

            // ── Intake ──
            if (gamepad1.circleWasPressed()) intakeOn = (intakeOn == 1) ? 0 : 1;
            if (gamepad1.squareWasPressed()) intakeOn = (intakeOn == 2) ? 0 : 2;
            switch (intakeOn) {
                case 1:  intakeMotor.setPower(-1); break;
                case 2:  intakeMotor.setPower( 1); break;
                default: intakeMotor.setPower( 0); break;
            }

            // ── Rapid fire: 3-shot burst on right trigger ──
            if (gamepad1.right_trigger > 0.3 && !isPushing && shots == 0) {
                shots = 3;
                blockServo.setPosition(BLOCK_SERVO_UP);
            }
            if (!isPushing && shots > 0) {
                isPushing = true;
                rapidFireTimer.reset();
                pushServo.setPosition(PUSH_SERVO_UP);
            }
            if (isPushing) {
                double t = rapidFireTimer.milliseconds();
                if (t <= 150) {
                    pushServo.setPosition(PUSH_SERVO_UP);
                } else if (t <= 300) {
                    pushServo.setPosition(PUSH_SERVO_DOWN);
                } else {
                    isPushing = false;
                    shots--;
                    if (shots == 0) blockServo.setPosition(BLOCK_SERVO_DOWN);
                }
            }

            // ── Manual single shot: right bumper ──
            if (gamepad1.rightBumperWasPressed() && !isPushingManual) {
                isPushingManual = true;
                pushTimer.reset();
                pushServo.setPosition(PUSH_SERVO_UP);
                blockServo.setPosition(BLOCK_SERVO_UP);
            }
            if (isPushingManual && pushTimer.milliseconds() > 400) {
                pushServo.setPosition(PUSH_SERVO_DOWN);
                blockServo.setPosition(BLOCK_SERVO_DOWN);
                isPushingManual = false;
            }

            // ── Telemetry ──
            double shootVel  = shootMotor.getVelocity();
            double shootErr  = targetVelocity - shootVel;

            telemetry.addData("Shooter/TargetVelocity",  targetVelocity);
            telemetry.addData("Shooter/ActualVelocity",  shootVel);
            telemetry.addData("Shooter/Error",           shootErr);
            telemetry.addData("Shooter/Power",           shootMotor.getPower());
            telemetry.addData("Shooter/AtTarget",        Math.abs(shootErr) < 50);

            telemetry.addData("Hood/Position",           hoodPos);
            telemetry.addData("Distance/in",             "%.1f", distanceFromRed());
            telemetry.addData("Shots/Queued",            shots);

            // Per-term breakdown for tuning
            telemetry.addData("PV/kS",           kS);
            telemetry.addData("PV/kV",           kV);
            telemetry.addData("PV/kP",           kP);
            telemetry.addData("PV/term_kV",      kV * targetVelocity);
            telemetry.addData("PV/term_kP_shoot", kP * shootErr);
            telemetry.update();
        }
    }

    // =========================================================
    //  PV Shooter Controller — matches NewTeleOp setShooterPV() exactly
    //
    //   power = kS  +  kV × target  +  kP × (target − actual)
    //
    //   kS      — overcomes static friction (shoot motor)
    //   kS_lift — overcomes static friction (lift motor, tune independently)
    //   kV      — open-loop feedforward; shared by both motors
    //   kP      — proportional correction; each motor uses its own velocity
    //
    //   Clamped to [0, 1] — flywheels never run in reverse
    // =========================================================
    private void setShooterPV(double targetRPM) {
        if (targetRPM <= 0) {
            shootMotor.setPower(0);
            shootMotor2.setPower(0);
            return;
        }

        double shootVel  = shootMotor.getVelocity();

        double shootPower = kS + (kV * targetRPM) + (kP * (targetRPM - shootVel));

        shootMotor.setPower(clamp(shootPower, -1, 1));
        shootMotor2.setPower(clamp(shootPower,  -1, 1));
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