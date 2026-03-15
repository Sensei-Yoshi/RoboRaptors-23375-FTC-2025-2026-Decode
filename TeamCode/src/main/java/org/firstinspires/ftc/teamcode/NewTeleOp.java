package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.Gamepad.LED_DURATION_CONTINUOUS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
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
    private DcMotorEx   shootMotor, intakeMotor, shootMotor2, liftMotor;
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
    public static double kS = 0.09;
    public static double kV = 0.0004;
    public static double kP = 0.01;
    public static double BLOCK_INTAKE_DELAY_MS = 300;

    // ─────────────────────────────────────────────
    //  Aim PIDF constants  (tune via Dashboard)
    // ─────────────────────────────────────────────
    public static double Kp_aim       = 0.008;
    public static double Ki_aim       = 0.0;
    public static double Kd_aim       = 0.0014;
    public static double Kf_aim       = 0.19;
    public static double aimTolerance = 1.23;

    // Max yaw output when distance > LONG_RANGE_AIM_THRESHOLD
    public static double AIM_MAX_YAW_LONG_RANGE   = 1.0;
    public static double LONG_RANGE_AIM_THRESHOLD = 110.0;

    private PIDFController aimPid;

    // ─────────────────────────────────────────────
    //  Shooter targets
    // ─────────────────────────────────────────────
    public static double IDLE_RPM  = 1100;
    public static double IDLE_HOOD = 0.520;

    public static double MANUAL_RPM  = 1100;
    public static double MANUAL_HOOD = 0.48;

    public static double rpmTolerance = 50;

    // ─────────────────────────────────────────────
    //  Block servo
    // ─────────────────────────────────────────────
    private static final double BLOCK_SERVO_DOWN     = 0.8;
    private static final double BLOCK_SERVO_UP       = 0.25;
    private static final double PUSH_SERVO_DOWN      = 0.9;
    public  static       double BLOCK_OPEN_DURATION_MS = 1000;

    public static double LONG_RANGE_INTAKE_SPEED = -0.85;

    private final ElapsedTime blockTimer        = new ElapsedTime();
    private final ElapsedTime blockServoUpTimer = new ElapsedTime();
    private boolean isBlocking    = false;
    private boolean intakeStarted = false;

    // ─────────────────────────────────────────────
    //  Lift constants  (tune via Dashboard)
    // ─────────────────────────────────────────────
    public static int    LIFT_UP_POSITION = -1150;
    public static double LIFT_POWER       = 1.0;

    private boolean liftUp = false;

    // ─────────────────────────────────────────────
    //  Limelight / distance state
    // ─────────────────────────────────────────────
    private static final long TARGET_HOLD_MS          = 30;
    private static final long CAMERA_BLOCK_TIMEOUT_MS = 400;

    private double lastTx                 = 0.0;
    private long   lastSeenTimeMs         = 0;
    private double lastDistance           = 0.0;
    private long   lastDistanceSeenTimeMs = 0;
    private long   lastValidTargetTime    = 0;

    private boolean cameraBlocked      = false;
    private boolean hasRumbledForBlock = false;

    // ─────────────────────────────────────────────
    //  Velocity lock
    // ─────────────────────────────────────────────
    private boolean velocityLocked = false;
    private double  lockedRPM      = 0.0;
    private double  lockedHood     = 0.0;

    // ─────────────────────────────────────────────
    //  Mode flags
    // ─────────────────────────────────────────────
    private boolean shooterOverride     = false;
    private boolean manualOverride      = false;
    private int     intakeOn            = 0;
    private int     preShootIntakeState = 0;

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

        aimPid = new PIDFController(new PIDFCoefficients(Kp_aim, Ki_aim, Kd_aim, Kf_aim));

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
                    aimPid.updateError(error);
                    aimPid.updateFeedForwardInput(Math.signum(error));

                    double rawYaw   = aimPid.run();
                    double yawLimit = (distance > LONG_RANGE_AIM_THRESHOLD)
                            ? AIM_MAX_YAW_LONG_RANGE
                            : 1.0;
                    yaw = clamp(rawYaw, -yawLimit, yawLimit);

                    if (Math.abs(error) < aimTolerance) {
                        yaw     = 0;
                        axial   = 0;
                        lateral = 0;
                        autoState = AutoState.SPINNING;
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
        shootMotor2 = hardwareMap.get(DcMotorEx.class, "shootMotor2");
        liftMotor   = hardwareMap.get(DcMotorEx.class, "liftMotor");

        hoodServo  = hardwareMap.get(Servo.class, "hoodServo");
        blockServo = hardwareMap.get(Servo.class, "blockServo");
        pushServo  = hardwareMap.get(Servo.class, "pushServo");
        light      = hardwareMap.get(Servo.class, "light");

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "right_back_drive");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // ── Drive motor directions ──
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // ── Zero-power behavior ──
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ── Shooter motor setup ──
        shootMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shootMotor2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shootMotor2.setDirection(DcMotorEx.Direction.REVERSE);

        // ── Lift motor setup ──
        liftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        liftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        // ── Servo starting positions ──
        hoodServo.setPosition(0.42);
        blockServo.setPosition(BLOCK_SERVO_DOWN);

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
                    LED_DURATION_CONTINUOUS);
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

        // Y (triangle): toggle lift up/down
        if (gamepad1.triangleWasPressed()) {
            liftUp = !liftUp;
            liftMotor.setTargetPosition(liftUp ? LIFT_UP_POSITION : 0);
            liftMotor.setPower(LIFT_POWER);
            liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        }

        // Circle/Square: intake toggle — only when not in a shoot sequence
        if (autoState == AutoState.IDLE) {
            if (gamepad1.circleWasPressed()) intakeOn = (intakeOn == 1) ? 0 : 1;
            if (gamepad1.squareWasPressed()) intakeOn = (intakeOn == 2) ? 0 : 2;

            switch (intakeOn) {
                case 1:  intakeMotor.setPower(-1); break;
                case 2:  intakeMotor.setPower( 1); break;
                default: intakeMotor.setPower( 0); break;
            }
        }
    }

    // =========================================================
    //  Shoot sequence initiation
    // =========================================================
    private double startShootSequence(double yaw) {
        if (manualOverride) {
            preShootIntakeState = intakeOn;
            intakeOn = 0;
            intakeMotor.setPower(0);
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
            preShootIntakeState = intakeOn;
            intakeOn = 0;
            intakeMotor.setPower(0);
            distance       = clampDistance(getStableDistance());
            velocityLocked = true;
            lockedRPM      = controlPointsRPM.get(distance);
            lockedHood     = controlPointsHood.get(distance);
            aimPid         = new PIDFController(new PIDFCoefficients(Kp_aim, Ki_aim, Kd_aim, Kf_aim));
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
    //   power = kS  +  kV × target  +  kP × (target − actual)
    // =========================================================
    private void setShooterPV(double targetRPM) {
        if (targetRPM <= 0 || shooterOverride) {
            shootMotor.setPower(0);
            shootMotor2.setPower(0);
            return;
        }
        double vel   = shootMotor.getVelocity();
        double power = kS + (kV * targetRPM) + (kP * (targetRPM - vel));
        shootMotor.setPower(clamp(power, -1, 1));
        shootMotor2.setPower(clamp(power, -1, 1));
    }

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
                    isBlocking    = false;
                    intakeStarted = false;
                    abortSequence("blocked during fire");
                    intakeOn = preShootIntakeState;
                    applyIntakePower();
                    break;
                }

                if (!isBlocking) {
                    isBlocking = true;
                    blockTimer.reset();
                    blockServoUpTimer.reset();
                    intakeStarted = false;
                    blockServo.setPosition(BLOCK_SERVO_UP);
                }

                if (!intakeStarted && blockServoUpTimer.milliseconds() >= BLOCK_INTAKE_DELAY_MS) {
                    intakeStarted = true;
                    intakeMotor.setPower(distance > LONG_RANGE_AIM_THRESHOLD
                            ? LONG_RANGE_INTAKE_SPEED
                            : -1.0);
                    intakeOn = 1;
                }

                if (blockTimer.milliseconds() >= BLOCK_OPEN_DURATION_MS) {
                    blockServo.setPosition(BLOCK_SERVO_DOWN);
                    isBlocking     = false;
                    intakeStarted  = false;
                    velocityLocked = false;
                    autoState      = AutoState.IDLE;
                    intakeOn       = preShootIntakeState;
                    applyIntakePower();
                }
                break;

            default:
                break;
        }
    }

    private void applyIntakePower() {
        switch (intakeOn) {
            case 1:  intakeMotor.setPower(-1); break;
            case 2:  intakeMotor.setPower( 1); break;
            default: intakeMotor.setPower( 0); break;
        }
    }

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

    private double clampDistance(double d) {
        if (d < 22)   return 23;
        if (d <= 85)  return d;
        if (d < 110)  return 79;
        if (d <= 135) return d;
        return 134;
    }

    // =========================================================
    //  Lookup table construction
    // =========================================================
    private void buildLookupTables() {
        controlPointsRPM.add(22,  1000);
        controlPointsRPM.add(25,  1000);
        controlPointsRPM.add(30,  1000);
        controlPointsRPM.add(35,  1100);
        controlPointsRPM.add(40,  1100);
        controlPointsRPM.add(45,  1100);
        controlPointsRPM.add(50,  1140);
        controlPointsRPM.add(55,  1150);
        controlPointsRPM.add(60,  1200);
        controlPointsRPM.add(65,  1200);
        controlPointsRPM.add(70,  1250);
        controlPointsRPM.add(75,  1250);
        controlPointsRPM.add(80,  1280);
        controlPointsRPM.add(85,  1280);
        controlPointsRPM.add(110, 1360);
        controlPointsRPM.add(115, 1370);
        controlPointsRPM.add(120, 1390);
        controlPointsRPM.add(125, 1390);
        controlPointsRPM.add(130, 1430);
        controlPointsRPM.add(135, 1450);
        controlPointsRPM.createLUT();

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
        controlPointsHood.add(70,  0.500);
        controlPointsHood.add(75,  0.500);
        controlPointsHood.add(80,  0.510);
        controlPointsHood.add(85,  0.510);
        controlPointsHood.add(110, 0.536);
        controlPointsHood.add(115, 0.540);
        controlPointsHood.add(120, 0.540);
        controlPointsHood.add(125, 0.540);
        controlPointsHood.add(130, 0.546);
        controlPointsHood.add(135, 0.568);
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

        telemetry.addData("State/AutoState",       autoState.toString());
        telemetry.addData("State/ManualOverride",   manualOverride);
        telemetry.addData("State/ShooterOFF",       shooterOverride);
        telemetry.addData("State/CameraBlocked",    cameraBlocked);
        telemetry.addData("State/VelocityLocked",   velocityLocked);

        telemetry.addData("Intake/State",           intakeOn);
        telemetry.addData("Intake/PreShootState",   preShootIntakeState);
        telemetry.addData("Intake/Power",           intakeMotor.getPower());

        telemetry.addData("Shooter/TargetRPM",      targetRPM);
        telemetry.addData("Shooter/ActualRPM",      actualRPM);
        telemetry.addData("Shooter/LiftRPM",        liftRPM);
        telemetry.addData("Shooter/LockedRPM",      lockedRPM);
        telemetry.addData("Shooter/RPM_Error",      rpmError);
        telemetry.addData("Shooter/AtTarget",       atRPMTarget(targetRPM));
        telemetry.addData("Shooter/ShootPower",     shootPower);
        telemetry.addData("Shooter/LiftPower",      shootMotor2.getPower());

        telemetry.addData("Hood/Target",            hoodTarget);
        telemetry.addData("Hood/Position",          hoodServo.getPosition());

        telemetry.addData("PV/kS",                  kS);
        telemetry.addData("PV/kV",                  kV);
        telemetry.addData("PV/kP",                  kP);
        telemetry.addData("PV/term_kS",             kS);
        telemetry.addData("PV/term_kV",             kV * targetRPM);
        telemetry.addData("PV/term_kP",             kP * rpmError);

        telemetry.addData("Aim/TX_degrees",         txAngle);
        telemetry.addData("Aim/Yaw_output",         yaw);
        telemetry.addData("Aim/Kp",                 Kp_aim);
        telemetry.addData("Aim/Ki",                 Ki_aim);
        telemetry.addData("Aim/Kd",                 Kd_aim);
        telemetry.addData("Aim/Kf",                 Kf_aim);
        telemetry.addData("Aim/Tolerance",          aimTolerance);
        telemetry.addData("Aim/MaxYaw_LongRange",   AIM_MAX_YAW_LONG_RANGE);
        telemetry.addData("Aim/LongRangeThreshold", LONG_RANGE_AIM_THRESHOLD);

        telemetry.addData("Distance/Raw_in",        distance);
        telemetry.addData("Distance/Clamped_in",    clampDistance(distance));

        telemetry.addData("Block/IsBlocking",       isBlocking);
        telemetry.addData("Block/TimerMs",          blockTimer.milliseconds());

        telemetry.addData("Lift/Up",                liftUp);
        telemetry.addData("Lift/TargetPos",         liftUp ? LIFT_UP_POSITION : 0);
        telemetry.addData("Lift/CurrentPos",        liftMotor.getCurrentPosition());
        telemetry.addData("Lift/Power",             liftMotor.getPower());

        telemetry.update();
    }

    // =========================================================
    //  Utility
    // =========================================================
    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}