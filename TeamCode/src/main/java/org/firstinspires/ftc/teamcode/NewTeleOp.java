package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.Gamepad.LED_DURATION_CONTINUOUS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.util.InterpLUT;
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
@TeleOp(name = "Red TeleOp", group = "Tuning")
public class NewTeleOp extends LinearOpMode {

    // ─────────────────────────────────────────────
    //  Auto-shoot state machine
    // ─────────────────────────────────────────────
    private enum AutoState { IDLE, AIMING, SPINNING, FIRING }
    private AutoState autoState = AutoState.IDLE;

    // ─────────────────────────────────────────────
    //  Hardware
    // ─────────────────────────────────────────────
    private DcMotorEx   shootMotor, intakeMotor, shootMotor2;
    private Servo       hoodServo, blockServo, pushServo, light;
    private DcMotor     leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;
    private Limelight3A limelight;

    // ─────────────────────────────────────────────
    //  Lookup tables  (distance → RPM / hood)
    // ─────────────────────────────────────────────
    private final InterpLUT controlPointsRPM  = new InterpLUT();
    private final InterpLUT controlPointsHood = new InterpLUT();

    // ─────────────────────────────────────────────
    //  Shooter PV constants  (tune via Dashboard)
    // ─────────────────────────────────────────────
    public static double kS      = 0.09;    // static / friction offset
    public static double kV      = 0.00038; // velocity feedforward (power per tick/s)
    public static double kP      = 0.01;    // proportional error gain
        // lift motor static offset (tune separately)

    // ─────────────────────────────────────────────
    //  Aim PID constants  (tune via Dashboard)
    // ─────────────────────────────────────────────
    public static double Kp_aim = 0.03;
    public static double Kd_aim = 0.0045;
    public static double Kf_aim = 0.18;    // sign-based static friction for rotation
    private PIDFController aimPid;

    // ─────────────────────────────────────────────
    //  Shooter targets
    // ─────────────────────────────────────────────
    public static double IDLE_RPM  = 1100;
    public static double IDLE_HOOD = 0.520;

    public static double MANUAL_RPM  = 1100;
    public static double MANUAL_HOOD = 0.48;

    public static double rpmTolerance = 50;
    public static double aimTolerance = 1.5;

    // ─────────────────────────────────────────────
    //  Block servo
    // ─────────────────────────────────────────────
    private static final double BLOCK_SERVO_DOWN     = 0.78;
    private static final double BLOCK_SERVO_UP       = 0.25;
    private static final double PUSH_SERVO_DOWN      = 0.9;
    public  static       double BLOCK_OPEN_DURATION_MS = 1000;

    private final ElapsedTime blockTimer = new ElapsedTime();
    private boolean isBlocking = false;

    // ─────────────────────────────────────────────
    //  Limelight / distance state
    // ─────────────────────────────────────────────
    private static final long TARGET_HOLD_MS           = 30;
    private static final long CAMERA_BLOCK_TIMEOUT_MS  = 400;

    private double lastTx                 = 0.0;
    private long   lastSeenTimeMs         = 0;
    private double lastDistance           = 0.0;
    private long   lastDistanceSeenTimeMs = 0;
    private long   lastValidTargetTime    = 0;

    private boolean cameraBlocked      = false;
    private boolean hasRumbledForBlock = false;

    // ─────────────────────────────────────────────
    //  Velocity lock  (snapshot on trigger press)
    // ─────────────────────────────────────────────
    private boolean velocityLocked = false;
    private double  lockedRPM      = 0.0;
    private double  lockedHood     = 0.0;

    // ─────────────────────────────────────────────
    //  Mode flags
    // ─────────────────────────────────────────────
    private boolean shooterOverride = false; // true = shooter OFF
    private boolean manualOverride  = false; // true = fixed RPM/hood
    private int     intakeOn        = 0;     // 0=off, 1=in, 2=out

    // ─────────────────────────────────────────────
    //  Misc
    // ─────────────────────────────────────────────
    private double distance    = 0;
    private double lastAutoRPM = 0;

    // =========================================================
    //  runOpMode
    // =========================================================
    @Override
    public void runOpMode() {
        initHardware();
        buildLookupTables();

        aimPid = new PIDFController(Kp_aim, 0, Kd_aim, 0);
        aimPid.setTolerance(aimTolerance);
        aimPid.setSetPoint(0);

        // Send telemetry to both the Driver Station and FTC Dashboard simultaneously
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            handleButtons();
            updateCameraBlockStatus();

            double axial   = -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // ── Trigger: start shoot sequence ──
            if (gamepad1.right_trigger > 0.3 && autoState == AutoState.IDLE) {
                yaw = startShootSequence(yaw);
            }

            // ── AIMING state: rotate robot toward target ──
            if (autoState == AutoState.AIMING) {
                if (cameraBlocked) {
                    abortSequence("blocked during aim");
                } else {
                    double error = getTx(24);
                    yaw = (-aimPid.calculate(error)) + (Kf_aim * Math.signum(error));
                    if (aimPid.atSetPoint()) {
                        autoState = AutoState.SPINNING;
                        axial     = 0;
                        lateral   = 0;
                    }
                }
            }

            driveMecanum(axial, lateral, yaw);

            // ── Determine RPM / hood targets ──
            distance = clampDistance(getStableDistance());
            double targetRPM;
            double hoodTarget;

            if (velocityLocked) {
                targetRPM  = lockedRPM;
                hoodTarget = lockedHood;
            } else {
                targetRPM  = IDLE_RPM;
                hoodTarget = IDLE_HOOD;
            }

            // ── Apply shooter power via PV controller ──
            applyShooterPower(targetRPM, hoodTarget);

            // ── Auto-state machine (SPINNING / FIRING) ──
            runStateMachine(targetRPM);

            // ── Manual block servo (right bumper) ──
            handleManualBlock();

            updateTelemetry(targetRPM, hoodTarget, yaw);
        }
    }

    // =========================================================
    //  Initialisation
    // =========================================================
    private void initHardware() {
        shootMotor  = hardwareMap.get(DcMotorEx.class, "shootMotor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        shootMotor2   = hardwareMap.get(DcMotorEx.class, "liftMotor");

        hoodServo  = hardwareMap.get(Servo.class, "hoodServo");
        blockServo = hardwareMap.get(Servo.class, "blockServo");
        pushServo  = hardwareMap.get(Servo.class, "pushServo");
        light      = hardwareMap.get(Servo.class, "light");

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "right_back_drive");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Drive direction
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Drive brake
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Shooter motors — RUN_WITHOUT_ENCODER so we drive power manually.
        // DcMotorEx.getVelocity() still works regardless of run mode.
        shootMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shootMotor2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shootMotor2.setDirection(DcMotorEx.Direction.REVERSE);

        // Servo starting positions
        hoodServo.setPosition(0.40);
        blockServo.setPosition(BLOCK_SERVO_DOWN);


        // Limelight
        limelight.start();
        limelight.pipelineSwitch(0);
    }

    // =========================================================
    //  Gamepad button handling
    // =========================================================
    private void handleButtons() {
        // A: toggle shooter on/off
        if (gamepad1.aWasPressed()) {
            shooterOverride = !shooterOverride;
            gamepad1.setLedColor(
                    shooterOverride ? 0 : 0,
                    shooterOverride ? 1 : 0,
                    shooterOverride ? 0 : 1,
                    LED_DURATION_CONTINUOUS); // green=OFF, blue=ON
        }

        // D-pad up: toggle manual override
        if (gamepad1.dpadUpWasPressed()) {
            manualOverride = !manualOverride;
            if (manualOverride) {
                gamepad1.setLedColor(1, 1, 0, LED_DURATION_CONTINUOUS); // yellow
            } else {
                limelight.start();
                gamepad1.setLedColor(0, 0, 1, LED_DURATION_CONTINUOUS); // blue
            }
        }

        // Circle/Square: intake toggle
        if (gamepad1.circleWasPressed()) intakeOn = (intakeOn == 1) ? 0 : 1;
        if (gamepad1.squareWasPressed()) intakeOn = (intakeOn == 2) ? 0 : 2;

        switch (intakeOn) {
            case 1:  intakeMotor.setPower(-1); break;
            case 2:  intakeMotor.setPower( 1); break;
            default: intakeMotor.setPower( 0); break;
        }
    }

    // =========================================================
    //  Shoot sequence initiation
    //  Returns (possibly modified) yaw for this loop iteration.
    // =========================================================
    private double startShootSequence(double yaw) {
        if (manualOverride) {
            velocityLocked = true;
            lockedRPM      = MANUAL_RPM;
            lockedHood     = MANUAL_HOOD;
            autoState      = AutoState.SPINNING;
        } else if (cameraBlocked) {
            if (!hasRumbledForBlock) {
                gamepad1.rumble(1.0, 1.0, 500);
                hasRumbledForBlock = true;
            }
        } else {
            distance           = clampDistance(getStableDistance());
            velocityLocked     = true;
            lockedRPM          = controlPointsRPM.get(distance);
            lockedHood         = controlPointsHood.get(distance);
            autoState          = AutoState.AIMING;
            hasRumbledForBlock = false;
        }
        return yaw;
    }

    // =========================================================
    //  Drive
    // =========================================================
    private void driveMecanum(double axial, double lateral, double yaw) {
        double d = Math.max(Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw), 1);
        leftFrontDrive.setPower((axial + lateral + yaw) / d);
        rightFrontDrive.setPower((axial - lateral - yaw) / d);
        leftBackDrive.setPower((axial - lateral + yaw) / d);
        rightBackDrive.setPower((axial + lateral - yaw) / d);
    }

    // =========================================================
    //  PV Shooter controller
    // =========================================================

    /**
     * Computes and applies motor power using a feedforward + proportional (PV) controller:
     *
     *   power = kS  +  kV × target  +  kP × (target − actual)
     *
     *   kS  — overcomes static friction so the motor doesn't stall at startup
     *   kV  — open-loop feedforward; maps target velocity directly to approximate power
     *   kP  — proportional term that closes the loop on remaining velocity error
     *
     * Because kV handles most of the work, kP only needs to correct small residuals,
     * so no integrator (I) or derivative (D) term is necessary for flywheel control.
     */
    private void setShooterPV(double targetRPM) {
        if (targetRPM <= 0 || shooterOverride) {
            shootMotor.setPower(0);
            shootMotor2.setPower(0);
            return;
        }

        double shootVel  = shootMotor.getVelocity();


        double shootPower = kS + (kV * targetRPM) + (kP * (targetRPM - shootVel));

        shootMotor.setPower(clamp(shootPower, -1, 1));
        shootMotor2.setPower(clamp(shootPower,  -1, 1));
    }

    /** Returns true when shooter velocity is within rpmTolerance of target. */
    private boolean atRPMTarget(double targetRPM) {
        return Math.abs(shootMotor.getVelocity() - targetRPM) < rpmTolerance;
    }

    private void applyShooterPower(double targetRPM, double hoodTarget) {
        if (manualOverride) {
            setShooterPV(MANUAL_RPM);
            hoodServo.setPosition(MANUAL_HOOD);
        } else if (!shooterOverride) {
            setShooterPV(targetRPM);
            hoodServo.setPosition(hoodTarget);
            lastAutoRPM = targetRPM;
        } else {
            setShooterPV(0);
        }
    }

    // =========================================================
    //  Auto-shoot state machine  (SPINNING / FIRING)
    // =========================================================
    private void runStateMachine(double targetRPM) {
        switch (autoState) {

            case SPINNING:
                if (cameraBlocked && !manualOverride) {
                    abortSequence("blocked during spin");
                } else if (atRPMTarget(targetRPM)) {
                    autoState = AutoState.FIRING;
                }
                break;

            case FIRING:
                if (cameraBlocked && !manualOverride) {
                    blockServo.setPosition(BLOCK_SERVO_DOWN);
                    isBlocking = false;
                    abortSequence("blocked during fire");
                    break;
                }
                // Open block servo once at entry
                if (!isBlocking) {
                    isBlocking = true;
                    blockTimer.reset();
                    blockServo.setPosition(BLOCK_SERVO_UP);
                }
                // Close after duration and return to IDLE
                if (blockTimer.milliseconds() >= BLOCK_OPEN_DURATION_MS) {
                    blockServo.setPosition(BLOCK_SERVO_DOWN);
                    isBlocking     = false;
                    velocityLocked = false;
                    autoState      = AutoState.IDLE;
                }
                break;

            default:
                break;
        }
    }

    /** Cancel any active shoot sequence and return shooter to idle. */
    private void abortSequence(String reason) {
        if (!hasRumbledForBlock) {
            gamepad1.rumble(1.0, 1.0, 500);
            hasRumbledForBlock = true;
        }
        autoState      = AutoState.IDLE;
        velocityLocked = false;
        isBlocking     = false;
        blockServo.setPosition(BLOCK_SERVO_DOWN);
        telemetry.addData("Abort", reason);
    }

    // =========================================================
    //  Manual block servo  (right bumper)
    // =========================================================
    private void handleManualBlock() {
        if (gamepad1.rightBumperWasPressed() && !isBlocking && autoState == AutoState.IDLE) {
            isBlocking = true;
            blockTimer.reset();
            blockServo.setPosition(BLOCK_SERVO_UP);
        }
        if (isBlocking && autoState == AutoState.IDLE
                && blockTimer.milliseconds() >= BLOCK_OPEN_DURATION_MS) {
            blockServo.setPosition(BLOCK_SERVO_DOWN);
            isBlocking = false;
        }
    }

    // =========================================================
    //  Camera / limelight helpers
    // =========================================================
    private void updateCameraBlockStatus() {
        long now = System.currentTimeMillis();
        if (now - lastValidTargetTime <= CAMERA_BLOCK_TIMEOUT_MS) {
            cameraBlocked      = false;
            hasRumbledForBlock = false;
        } else {
            cameraBlocked = true;
        }
    }

    private double getTx(double targetID) {
        LLResult result = limelight.getLatestResult();
        if (result == null) return holdTx();

        long now = System.currentTimeMillis();
        for (LLResultTypes.FiducialResult f : result.getFiducialResults()) {
            if (f != null && f.getFiducialId() == targetID) {
                lastSeenTimeMs      = now;
                lastValidTargetTime = now;
                cameraBlocked       = false;
                lastTx              = f.getTargetXDegrees();
                return lastTx;
            }
        }
        return holdTx();
    }

    private double holdTx() {
        return (System.currentTimeMillis() - lastSeenTimeMs <= TARGET_HOLD_MS) ? lastTx : 0;
    }

    private double distanceFromTag(double tagID) {
        LLResult result = limelight.getLatestResult();
        if (result == null) { light.setPosition(0.388); return 0.0; }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials.isEmpty()) { light.setPosition(0.388); return 0.0; }

        for (LLResultTypes.FiducialResult f : fiducials) {
            if (f != null && f.getFiducialId() == tagID) {
                light.setPosition(0.728);
                double x = f.getCameraPoseTargetSpace().getPosition().x / DistanceUnit.mPerInch;
                double z = f.getCameraPoseTargetSpace().getPosition().z / DistanceUnit.mPerInch;
                Vector v = new Vector();
                v.setOrthogonalComponents(x, z);
                return v.getMagnitude();
            }
        }
        return 0.0;
    }

    private double getStableDistance() {
        double d   = distanceFromTag(24);
        long   now = System.currentTimeMillis();
        if (d > 0) {
            lastDistance           = d;
            lastDistanceSeenTimeMs = now;
            return d;
        }
        if (now - lastDistanceSeenTimeMs <= TARGET_HOLD_MS) return lastDistance;
        return 0;
    }

    /**
     * Clamps raw distance into the two calibrated LUT ranges,
     * holding at the edge values across the dead-band gap (~85–110 in).
     */
    private double clampDistance(double d) {
        if (d < 22)   return 23;
        if (d <= 85)  return d;
        if (d < 110)  return 79;   // hold at last close-range point
        if (d <= 135) return d;
        return 134;
    }

    // =========================================================
    //  Lookup table construction
    // =========================================================
    private void buildLookupTables() {
        // RPM vs distance (inches)
        controlPointsRPM.add(22,  1100);
        controlPointsRPM.add(25,  1100);
        controlPointsRPM.add(30,  1100);
        controlPointsRPM.add(35,  1100);
        controlPointsRPM.add(40,  1150);
        controlPointsRPM.add(45,  1150);
        controlPointsRPM.add(50,  1180);
        controlPointsRPM.add(55,  1180);
        controlPointsRPM.add(60,  1280);
        controlPointsRPM.add(65,  1280);
        controlPointsRPM.add(70,  1380);
        controlPointsRPM.add(75,  1380);
        controlPointsRPM.add(80,  1430);
        controlPointsRPM.add(85,  1430);
        controlPointsRPM.add(110, 1560);
        controlPointsRPM.add(115, 1560);
        controlPointsRPM.add(120, 1620);
        controlPointsRPM.add(125, 1620);
        controlPointsRPM.add(130, 1650);
        controlPointsRPM.add(135, 1680);
        controlPointsRPM.createLUT();

        // Hood position vs distance (inches)
        controlPointsHood.add(22,  0.482);
        controlPointsHood.add(25,  0.482);
        controlPointsHood.add(30,  0.482);
        controlPointsHood.add(35,  0.482);
        controlPointsHood.add(40,  0.480);
        controlPointsHood.add(45,  0.480);
        controlPointsHood.add(50,  0.482);
        controlPointsHood.add(55,  0.486);
        controlPointsHood.add(60,  0.486);
        controlPointsHood.add(65,  0.495);
        controlPointsHood.add(70,  0.498);
        controlPointsHood.add(75,  0.498);
        controlPointsHood.add(80,  0.510);
        controlPointsHood.add(85,  0.510);
        controlPointsHood.add(110, 0.522);
        controlPointsHood.add(115, 0.522);
        controlPointsHood.add(120, 0.535);
        controlPointsHood.add(125, 0.535);
        controlPointsHood.add(130, 0.528);
        controlPointsHood.add(135, 0.526);
        controlPointsHood.createLUT();
    }

    // =========================================================
    //  Telemetry
    // =========================================================
    private void updateTelemetry(double targetRPM, double hoodTarget, double yaw) {
        double actualRPM  = shootMotor.getVelocity();
        double rpmError   = targetRPM - actualRPM;
        double shootPower = shootMotor.getPower();
        double liftRPM    = shootMotor2.getVelocity();
        double txAngle    = getTx(24);

        // ── State ──────────────────────────────────────────────────────────
        // Dashboard renders strings as labels, not graphs — fine for mode flags.
        telemetry.addData("State/AutoState",      autoState.toString());
        telemetry.addData("State/ManualOverride",  manualOverride);
        telemetry.addData("State/ShooterOFF",      shooterOverride);
        telemetry.addData("State/CameraBlocked",   cameraBlocked);
        telemetry.addData("State/VelocityLocked",  velocityLocked);

        // ── Shooter — these show as live graphs on the Dashboard ───────────
        telemetry.addData("Shooter/TargetRPM",     targetRPM);
        telemetry.addData("Shooter/ActualRPM",     actualRPM);
        telemetry.addData("Shooter/LiftRPM",       liftRPM);
        telemetry.addData("Shooter/LockedRPM",     lockedRPM);
        telemetry.addData("Shooter/RPM_Error",     rpmError);
        telemetry.addData("Shooter/AtTarget",      atRPMTarget(targetRPM));
        telemetry.addData("Shooter/ShootPower",    shootPower);
        telemetry.addData("Shooter/LiftPower",     shootMotor2.getPower());

        // ── Hood ───────────────────────────────────────────────────────────
        telemetry.addData("Hood/Target",           hoodTarget);
        telemetry.addData("Hood/Position",         hoodServo.getPosition());

        // ── PV Gains — live view confirms Dashboard config changes applied ─
        telemetry.addData("PV/kS",                 kS);
        telemetry.addData("PV/kV",                 kV);
        telemetry.addData("PV/kP",                 kP);
        // Computed contributions (useful for understanding each term's weight)
        telemetry.addData("PV/term_kS",            kS);
        telemetry.addData("PV/term_kV",            kV * targetRPM);
        telemetry.addData("PV/term_kP",            kP * rpmError);

        // ── Aim PID ────────────────────────────────────────────────────────
        telemetry.addData("Aim/TX_degrees",        txAngle);
        telemetry.addData("Aim/Yaw_output",        yaw);
        telemetry.addData("Aim/Kp",                Kp_aim);
        telemetry.addData("Aim/Kd",                Kd_aim);
        telemetry.addData("Aim/Kf",                Kf_aim);
        telemetry.addData("Aim/AtSetPoint",        aimPid.atSetPoint());

        // ── Distance ───────────────────────────────────────────────────────
        telemetry.addData("Distance/Raw_in",       distance);
        telemetry.addData("Distance/Clamped_in",   clampDistance(distance));

        // ── Block servo ────────────────────────────────────────────────────
        telemetry.addData("Block/IsBlocking",      isBlocking);
        telemetry.addData("Block/TimerMs",         blockTimer.milliseconds());

        telemetry.update();
    }

    // =========================================================
    //  Utility
    // =========================================================
    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}