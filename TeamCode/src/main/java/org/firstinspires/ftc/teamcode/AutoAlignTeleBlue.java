package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.Gamepad.LED_DURATION_CONTINUOUS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsTeleOP;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp(name = "AutoAlignTeleBlue", group = "Tuning")
public class AutoAlignTeleBlue extends LinearOpMode {

    // ─────────────────────────────────────────────
    //  Auto-shoot state machine
    //  AIMING_AND_SPINNING replaces the old serial AIMING → SPINNING pair.
    //  Both heading alignment and RPM ramp-up now happen in parallel.
    // ─────────────────────────────────────────────
    private enum AutoState { IDLE, AIMING_AND_SPINNING, FIRING }
    private AutoState autoState = AutoState.IDLE;

    // ─────────────────────────────────────────────
    //  Hardware
    // ─────────────────────────────────────────────
    private DcMotorEx   shootMotor, intakeMotor, shootMotor2, liftMotor;
    private Servo       hoodServo, blockServo, pushServo, light;
    private DcMotor     leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;
    private Limelight3A limelight;
    private Follower    follower;

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
    public static double BLOCK_INTAKE_DELAY_MS = 150;

    // ─────────────────────────────────────────────
    //  Shoot target — field coords in Pedro inches
    //  RED:  (138, 138)   BLUE: (6, 138)
    // ─────────────────────────────────────────────
    public static double SHOOT_TARGET_X = 6.0;
    public static double SHOOT_TARGET_Y = 138.0;

    // ─────────────────────────────────────────────
    //  Aim PIDF constants  (tune via Dashboard)
    //  Error is in RADIANS — start Kp ~50x your old TX-based value
    // ─────────────────────────────────────────────
    public static double Kp_aim           = 0.86;
    public static double Ki_aim           = 0.0;
    public static double Kd_aim           = 0.09;
    public static double Kf_aim           = 0.18;
    public static double headingTolerance = 0.015; // ~1.7°

    public static double AIM_MAX_YAW_LONG_RANGE   = 1.0;
    public static double LONG_RANGE_AIM_THRESHOLD = 110.0;

    private PIDFController aimPid;

    // ─────────────────────────────────────────────
    //  Cached PIDF coefficients
    //  Only reallocates PIDFCoefficients when Dashboard values actually change,
    //  eliminating per-loop object allocation and garbage collection.
    // ─────────────────────────────────────────────
    private double cachedKp = Double.NaN;
    private double cachedKi = Double.NaN;
    private double cachedKd = Double.NaN;
    private double cachedKf = Double.NaN;

    // ─────────────────────────────────────────────
    //  Shooter targets
    // ─────────────────────────────────────────────
    public static double IDLE_RPM  = 1200;
    public static double IDLE_HOOD = 0.520;

    public static double MANUAL_RPM  = 1100;
    public static double MANUAL_HOOD = 0.48;

    public static double rpmTolerance = 30;

    // ─────────────────────────────────────────────
    //  Block servo
    // ─────────────────────────────────────────────
    private static final double BLOCK_SERVO_DOWN       = 0.8;
    private static final double BLOCK_SERVO_UP         = 0.25;
    public  static       double BLOCK_OPEN_DURATION_MS = 1000;

    public static double LONG_RANGE_INTAKE_SPEED = -0.82;

    private final ElapsedTime blockTimer        = new ElapsedTime();
    private final ElapsedTime blockServoUpTimer = new ElapsedTime();
    private boolean isBlocking    = false;
    private boolean intakeStarted = false;

    // ─────────────────────────────────────────────
    //  Lift
    // ─────────────────────────────────────────────
    public static int    LIFT_UP_POSITION = -1150;
    public static double LIFT_POWER       = 1.0;
    private boolean liftUp = false;

    // ─────────────────────────────────────────────
    //  Camera block detection
    // ─────────────────────────────────────────────
    private static final long CAMERA_BLOCK_TIMEOUT_MS = 400;
    private long    lastValidTargetTime = 0;
    private boolean cameraBlocked       = false;
    private boolean hasRumbledForBlock  = false;

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
    //  Hood servo position cache
    //  Avoids redundant I2C servo writes when the position hasn't changed.
    // ─────────────────────────────────────────────
    private double lastHoodPosition = -1.0;

    // ─────────────────────────────────────────────
    //  Per-loop cached values — updated once at the top of each loop
    //  iteration so all methods share the same snapshot.
    // ─────────────────────────────────────────────
    private double currentVelocity = 0.0;  // shootMotor.getVelocity()
    private double targetHeading   = 0.0;  // computeTargetHeading(robotPose)

    // ─────────────────────────────────────────────
    //  AprilTag filter — only accept botpose when this tag is visible
    // ─────────────────────────────────────────────
    private static final int TARGET_TAG_ID = 20; // Blue alliance

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

        aimPid    = new PIDFController(new PIDFCoefficients(Kp_aim, Ki_aim, Kd_aim, Kf_aim));
        cachedKp  = Kp_aim;  cachedKi = Ki_aim;
        cachedKd  = Kd_aim;  cachedKf = Kf_aim;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            follower.update();

            // ── Cache expensive reads once per loop ──────────────────────
            Pose   robotPose = follower.getPose();           // single getPose() call
            currentVelocity  = shootMotor.getVelocity();     // single encoder read
            targetHeading    = computeTargetHeading(robotPose); // single trig computation
            // ─────────────────────────────────────────────────────────────

            handleButtons();
            updateCameraBlockStatus();
            updateLight();

            double axial   = -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // ── Pre-spin in IDLE ──────────────────────────────────────────
            // Continuously spin the shooter to the distance-predicted RPM so
            // the trigger response is near-instant (RPM is already settled).
            distance = clampDistance(getStableDistance(robotPose));
            if (autoState == AutoState.IDLE && !shooterOverride && !manualOverride) {
                lockedRPM      = controlPointsRPM.get(distance);
                lockedHood     = controlPointsHood.get(distance);
                velocityLocked = true;
            }
            // ─────────────────────────────────────────────────────────────

            // ── Trigger: start shoot sequence ─────────────────────────────
            if (gamepad1.right_trigger > 0.3 && autoState == AutoState.IDLE) {
                yaw = startShootSequence(yaw);
            }

            // ── AIMING_AND_SPINNING: rotate + wait for RPM simultaneously ─
            if (autoState == AutoState.AIMING_AND_SPINNING) {
                double headingError = MathFunctions.getTurnDirection(
                        robotPose.getHeading(), targetHeading)
                        * MathFunctions.getSmallestAngleDifference(
                        robotPose.getHeading(), targetHeading);

                // Update PIDF only when Dashboard tuning values change
                if (Kp_aim != cachedKp || Ki_aim != cachedKi
                        || Kd_aim != cachedKd || Kf_aim != cachedKf) {
                    aimPid.setCoefficients(new PIDFCoefficients(Kp_aim, Ki_aim, Kd_aim, Kf_aim));
                    cachedKp = Kp_aim; cachedKi = Ki_aim;
                    cachedKd = Kd_aim; cachedKf = Kf_aim;
                }

                aimPid.updateError(headingError);
                aimPid.updateFeedForwardInput(Math.signum(headingError));

                double yawLimit = (distance > LONG_RANGE_AIM_THRESHOLD) ? AIM_MAX_YAW_LONG_RANGE : 1.0;
                yaw = clamp(-aimPid.run(), -yawLimit, yawLimit);

                // Fire only when BOTH heading AND RPM are on target
                boolean headingReady = Math.abs(headingError) < headingTolerance;
                boolean rpmReady     = atRPMTarget(lockedRPM);

                if (headingReady && rpmReady) {
                    yaw       = 0;
                    axial     = 0;
                    lateral   = 0;
                    autoState = AutoState.FIRING;
                }
            }
            // ─────────────────────────────────────────────────────────────

            driveMecanum(axial, lateral, yaw);

            // ── Shooter power ─────────────────────────────────────────────
            double targetRPM  = velocityLocked ? lockedRPM  : IDLE_RPM;
            double hoodTarget = velocityLocked ? lockedHood : IDLE_HOOD;

            applyShooterPower(targetRPM, hoodTarget);
            runStateMachine();
            handleManualBlock();
            updateTelemetry(targetRPM, hoodTarget, yaw, robotPose);
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

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shootMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shootMotor2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shootMotor2.setDirection(DcMotorEx.Direction.REVERSE);

        liftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        liftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        hoodServo.setPosition(0.42);
        blockServo.setPosition(BLOCK_SERVO_DOWN);

        follower = ConstantsTeleOP.createFollower(hardwareMap);

        limelight.start();
        limelight.pipelineSwitch(0);
    }

    // =========================================================
    //  Gamepad button handling
    // =========================================================
    private void handleButtons() {
        if (gamepad1.aWasPressed()) {
            shooterOverride = !shooterOverride;
            gamepad1.setLedColor(
                    shooterOverride ? 0 : 0,
                    shooterOverride ? 1 : 0,
                    shooterOverride ? 0 : 1,
                    LED_DURATION_CONTINUOUS);
        }

        if (gamepad1.dpadUpWasPressed()) {
            manualOverride = !manualOverride;
            if (manualOverride) {
                gamepad1.setLedColor(1, 1, 0, LED_DURATION_CONTINUOUS);
            } else {
                gamepad1.setLedColor(0, 0, 1, LED_DURATION_CONTINUOUS);
            }
        }

        // Left bumper: relocalize Pedro from Limelight mt1 botpose
        if (gamepad1.leftBumperWasPressed()) {
            relocalize();
        }

        if (gamepad1.triangleWasPressed()) {
            liftUp = !liftUp;
            liftMotor.setTargetPosition(liftUp ? LIFT_UP_POSITION : 0);
            liftMotor.setPower(LIFT_POWER);
            liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        }

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
        preShootIntakeState = intakeOn;
        intakeOn = 0;
        intakeMotor.setPower(0);

        if (manualOverride) {
            velocityLocked = true;
            lockedRPM      = MANUAL_RPM;
            lockedHood     = MANUAL_HOOD;
        }
        // In auto mode lockedRPM/lockedHood are already set by pre-spin —
        // no need to re-query the LUT here.

        // Reset PID integrator for a clean aim
        aimPid    = new PIDFController(new PIDFCoefficients(Kp_aim, Ki_aim, Kd_aim, Kf_aim));
        cachedKp  = Kp_aim; cachedKi = Ki_aim;
        cachedKd  = Kd_aim; cachedKf = Kf_aim;

        hasRumbledForBlock = false;
        autoState          = AutoState.AIMING_AND_SPINNING;
        return yaw;
    }

    // =========================================================
    //  Drive
    //  ensureBrake() guards each motor write so we only pay the I2C
    //  cost when Pedro's follower.update() has flipped a motor to FLOAT.
    // =========================================================
    private void driveMecanum(double axial, double lateral, double yaw) {
        ensureBrake(leftFrontDrive);
        ensureBrake(leftBackDrive);
        ensureBrake(rightFrontDrive);
        ensureBrake(rightBackDrive);
        double d = Math.max(Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw), 1);
        leftFrontDrive.setPower( (axial + lateral + yaw) / d);
        rightFrontDrive.setPower((axial - lateral - yaw) / d);
        leftBackDrive.setPower(  (axial - lateral + yaw) / d);
        rightBackDrive.setPower( (axial + lateral - yaw) / d);
    }

    /** Writes BRAKE only if Pedro flipped this motor back to FLOAT — saves I2C bandwidth. */
    private void ensureBrake(DcMotor motor) {
        if (motor.getZeroPowerBehavior() != DcMotor.ZeroPowerBehavior.BRAKE) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    // =========================================================
    //  Compute field-relative heading from robot pose to shoot target
    // =========================================================
    private double computeTargetHeading(Pose robotPose) {
        double dx = SHOOT_TARGET_X - robotPose.getX();
        double dy = SHOOT_TARGET_Y - robotPose.getY();
        return Math.atan2(dy, dx);
    }

    // =========================================================
    //  Relocalization — write Limelight mt1 botpose into Pedro
    // =========================================================
    private void relocalize() {
        Pose llPose = getRobotPosFromTarget();
        if (llPose != null) {
            follower.setPose(llPose);
            telemetry.addData("Relocalize", "OK → (%.1f, %.1f, %.1f°)",
                    llPose.getX(), llPose.getY(), Math.toDegrees(llPose.getHeading()));
        } else {
            telemetry.addData("Relocalize", "FAILED — no valid mt1 result");
        }
    }

    private Pose getRobotPosFromTarget() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;

        // Only relocalize when our specific tag (ID 20, Blue) is visible
        boolean tagVisible = false;
        for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
            if (fiducial.getFiducialId() == TARGET_TAG_ID) {
                tagVisible = true;
                break;
            }
        }
        if (!tagVisible) return null;

        Pose3D robotPos = result.getBotpose();
        double angle = robotPos.getOrientation().getYaw(AngleUnit.DEGREES) - 90;
        if (angle > 360) angle -= 360;
        return new Pose(
                robotPos.getPosition().y / 0.0254 + 70.625,
                -robotPos.getPosition().x / 0.0254 + 70.625,
                Math.toRadians(angle));
    }

    // =========================================================
    //  PV Shooter controller  (uses cached currentVelocity — no extra I2C read)
    // =========================================================
    private void setShooterPV(double targetRPM) {
        if (targetRPM <= 0 || shooterOverride) {
            shootMotor.setPower(0);
            shootMotor2.setPower(0);
            return;
        }
        double power = kS + (kV * targetRPM) + (kP * (targetRPM - currentVelocity));
        shootMotor.setPower(clamp(power, -1, 1));
        shootMotor2.setPower(clamp(power, -1, 1));
    }

    /** Uses cached currentVelocity — no extra motor read. */
    private boolean atRPMTarget(double targetRPM) {
        return Math.abs(currentVelocity - targetRPM) < rpmTolerance;
    }

    private void applyShooterPower(double targetRPM, double hoodTarget) {
        if (manualOverride) {
            setShooterPV(MANUAL_RPM);
            setHoodPosition(MANUAL_HOOD);
        } else if (!shooterOverride) {
            setShooterPV(targetRPM);
            setHoodPosition(hoodTarget);
            lastAutoRPM = targetRPM;
        } else {
            setShooterPV(0);
        }
    }

    /** Guards hood servo writes — only sends I2C command when position actually changes. */
    private void setHoodPosition(double position) {
        if (Math.abs(position - lastHoodPosition) > 0.001) {
            hoodServo.setPosition(position);
            lastHoodPosition = position;
        }
    }

    // =========================================================
    //  Auto-shoot state machine
    // =========================================================
    private void runStateMachine() {
        if (autoState != AutoState.FIRING) return;

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
                    ? LONG_RANGE_INTAKE_SPEED : -1.0);
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
    }

    private void applyIntakePower() {
        switch (intakeOn) {
            case 1:  intakeMotor.setPower(-1); break;
            case 2:  intakeMotor.setPower( 1); break;
            default: intakeMotor.setPower( 0); break;
        }
    }

    // =========================================================
    //  Manual block servo
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
    //  Light — yellow when AprilTag not detected, purple when detected
    // =========================================================
    private void updateLight() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult f : result.getFiducialResults()) {
                if (f != null && f.getFiducialId() == TARGET_TAG_ID) {
                    light.setPosition(0.728); // purple
                    return;
                }
            }
        }
        light.setPosition(0.388); // yellow
    }

    // =========================================================
    //  Camera block detection
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

    // =========================================================
    //  Distance helpers  (accepts cached Pose — no extra getPose() call)
    // =========================================================
    private double getStableDistance(Pose robotPose) {
        double dx = SHOOT_TARGET_X - robotPose.getX();
        double dy = SHOOT_TARGET_Y - robotPose.getY();
        return Math.hypot(dx, dy);
    }

    private double clampDistance(double d) {
        if (d < 22)   return 23;
        if (d <= 85)  return d;
        if (d < 110)  return 79;
        if (d <= 140) return d;
        return 139;
    }

    // =========================================================
    //  Lookup tables
    // =========================================================
    private void buildLookupTables() {
        controlPointsRPM.add(22,  1000);
        controlPointsRPM.add(25,  1000);
        controlPointsRPM.add(30,  1000);
        controlPointsRPM.add(35,  1050);
        controlPointsRPM.add(40,  1050);
        controlPointsRPM.add(45,  1070);
        controlPointsRPM.add(50,  1070);
        controlPointsRPM.add(55,  1070);
        controlPointsRPM.add(60,  1070);
        controlPointsRPM.add(65,  1070);
        controlPointsRPM.add(70,  1200);
        controlPointsRPM.add(75,  1200);
        controlPointsRPM.add(80,  1230);
        controlPointsRPM.add(85,  1230);
        controlPointsRPM.add(115, 1370);
        controlPointsRPM.add(120, 1370);
        controlPointsRPM.add(125, 1390);
        controlPointsRPM.add(130, 1390);
        controlPointsRPM.add(135, 1400);
        controlPointsRPM.add(140, 1400);
        controlPointsRPM.createLUT();

        controlPointsHood.add(22,  0.482);
        controlPointsHood.add(25,  0.482);
        controlPointsHood.add(30,  0.482);
        controlPointsHood.add(35,  0.482);
        controlPointsHood.add(40,  0.482);
        controlPointsHood.add(45,  0.482);
        controlPointsHood.add(50,  0.482);
        controlPointsHood.add(55,  0.486);
        controlPointsHood.add(60,  0.486);
        controlPointsHood.add(65,  0.495);
        controlPointsHood.add(70,  0.497);
        controlPointsHood.add(75,  0.497);
        controlPointsHood.add(80,  0.500);
        controlPointsHood.add(85,  0.500);
        controlPointsHood.add(115, 0.540);
        controlPointsHood.add(120, 0.540);
        controlPointsHood.add(125, 0.541);
        controlPointsHood.add(130, 0.541);
        controlPointsHood.add(135, 0.542);
        controlPointsHood.add(140, 0.542);
        controlPointsHood.createLUT();
    }

    // =========================================================
    //  Telemetry
    //  Uses cached robotPose, currentVelocity, and targetHeading —
    //  no extra getPose(), getVelocity(), or trig calls here.
    // =========================================================
    private void updateTelemetry(double targetRPM, double hoodTarget, double yaw, Pose robotPose) {
        double rpmError     = targetRPM - currentVelocity;
        double headingError = MathFunctions.getTurnDirection(robotPose.getHeading(), targetHeading)
                * MathFunctions.getSmallestAngleDifference(robotPose.getHeading(), targetHeading);

        telemetry.addData("State/AutoState",         autoState.toString());
        telemetry.addData("State/ManualOverride",     manualOverride);
        telemetry.addData("State/ShooterOFF",         shooterOverride);
        telemetry.addData("State/CameraBlocked",      cameraBlocked);
        telemetry.addData("State/VelocityLocked",     velocityLocked);

        telemetry.addData("Shooter/TargetRPM",        targetRPM);
        telemetry.addData("Shooter/ActualRPM",        currentVelocity);
        telemetry.addData("Shooter/RPM_Error",        rpmError);
        telemetry.addData("Shooter/AtTarget",         atRPMTarget(targetRPM));
        telemetry.addData("Shooter/ShootPower",       shootMotor.getPower());
        telemetry.addData("Shooter/LiftRPM",          shootMotor2.getVelocity());

        telemetry.addData("Hood/Target",              hoodTarget);
        telemetry.addData("Hood/Position",            hoodServo.getPosition());

        telemetry.addData("PV/kS",                    kS);
        telemetry.addData("PV/kV",                    kV);
        telemetry.addData("PV/kP",                    kP);
        telemetry.addData("PV/term_kV",               kV * targetRPM);
        telemetry.addData("PV/term_kP",               kP * rpmError);

        telemetry.addData("Aim/TargetHeading_deg",    Math.toDegrees(targetHeading));
        telemetry.addData("Aim/CurrentHeading_deg",   Math.toDegrees(robotPose.getHeading()));
        telemetry.addData("Aim/HeadingError_deg",     Math.toDegrees(headingError));
        telemetry.addData("Heading Error",            headingError);
        telemetry.addData("Aim/Tolerance_deg",        Math.toDegrees(headingTolerance));
        telemetry.addData("Aim/Yaw_output",           yaw);
        telemetry.addData("Aim/Kp",                   Kp_aim);
        telemetry.addData("Aim/Kd",                   Kd_aim);
        telemetry.addData("Aim/MaxYaw_LongRange",     AIM_MAX_YAW_LONG_RANGE);
        telemetry.addData("Aim/LongRangeThreshold",   LONG_RANGE_AIM_THRESHOLD);

        telemetry.addData("Pose/X",                   robotPose.getX());
        telemetry.addData("Pose/Y",                   robotPose.getY());
        telemetry.addData("Pose/HeadingDeg",          Math.toDegrees(robotPose.getHeading()));
        telemetry.addData("Pose/ShootTarget_X",       SHOOT_TARGET_X);
        telemetry.addData("Pose/ShootTarget_Y",       SHOOT_TARGET_Y);

        telemetry.addData("Distance/in",              distance);  // already clamped

        telemetry.addData("Block/IsBlocking",         isBlocking);
        telemetry.addData("Block/TimerMs",            blockTimer.milliseconds());

        telemetry.addData("Lift/Up",                  liftUp);
        telemetry.addData("Lift/CurrentPos",          liftMotor.getCurrentPosition());

        telemetry.update();
    }

    // =========================================================
    //  Utility
    // =========================================================
    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}