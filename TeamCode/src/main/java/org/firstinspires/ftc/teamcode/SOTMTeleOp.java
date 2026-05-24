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
import com.pedropathing.math.MathFunctions;
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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

@Config
@TeleOp(name = "SOTM Tele", group = "Tuning")
public class SOTMTeleOp extends LinearOpMode {

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
    private Follower    follower;

    // ─────────────────────────────────────────────
    //  Physics constants  (tune via Dashboard)
    // ─────────────────────────────────────────────

    /** Gravity in inches/s² */
    public static double GRAVITY_IN_S2 = 386.1;

    /**
     * Height of the goal above the robot's launch point, in inches.
     * Positive = goal is above the shooter exit.
     */
    public static double GOAL_HEIGHT_IN = 26.0;

    /**
     * Required ball entry angle at the goal, in degrees.
     * Negative = ball arriving downward (e.g. -30 means 30° below horizontal).
     */
    public static double ENTRY_ANGLE_DEG = -30.0;

    /**
     * Pedro velocity units → inches/second scale factor.
     * Tune on Dashboard by observing VelComp/RobotSpeed_ips while driving at a known speed.
     */
    public static double PEDRO_VEL_TO_IPS = 1.0;

    // ─────────────────────────────────────────────
    //  Hood servo calibration  (section C of PDF)
    //  Servo Position = ((s1-s2)/(a1-a2)) * (alpha - a1) + s1
    // ─────────────────────────────────────────────

    /** Physical hood angle (degrees) at calibration point 1 — measure this */
    public static double HOOD_ANGLE_1_DEG  = 40.0;
    /** Servo position at HOOD_ANGLE_1_DEG — measure this */
    public static double HOOD_SERVO_POS_1  = 0.480;

    /** Physical hood angle (degrees) at calibration point 2 — measure this */
    public static double HOOD_ANGLE_2_DEG  = 60.0;
    /** Servo position at HOOD_ANGLE_2_DEG — measure this */
    public static double HOOD_SERVO_POS_2  = 0.542;

    /** Minimum physical hood angle the mechanism can reach (degrees) */
    public static double HOOD_MIN_ANGLE_DEG = 40.0;
    /** Maximum physical hood angle the mechanism can reach (degrees) */
    public static double HOOD_MAX_ANGLE_DEG = 60.0;

    // ─────────────────────────────────────────────
    //  Flywheel calibration  (section C of PDF)
    //  RPM = RPM_SLOPE * v0_in_per_s + RPM_INTERCEPT
    // ─────────────────────────────────────────────

    /** Slope of the RPM-vs-launch-speed line. Calibrate empirically. */
    public static double RPM_SLOPE     = 4.5;
    /** Intercept of the RPM-vs-launch-speed line. Calibrate empirically. */
    public static double RPM_INTERCEPT = 200.0;

    // ─────────────────────────────────────────────
    //  Toggle: physics calc vs legacy LUT
    // ─────────────────────────────────────────────
    public static boolean USE_PHYSICS_CALC = true;

    // ─────────────────────────────────────────────
    //  Shooter PV constants
    // ─────────────────────────────────────────────
    public static double kS = 0.09;
    public static double kV = 0.0004;
    public static double kP = 0.01;
    public static double BLOCK_INTAKE_DELAY_MS = 300;

    // ─────────────────────────────────────────────
    //  Shoot target — field coords in Pedro inches
    //  RED:  (138, 138)   BLUE: (6, 138)
    // ─────────────────────────────────────────────
    public static double SHOOT_TARGET_X = 138.0;
    public static double SHOOT_TARGET_Y = 138.0;

    // ─────────────────────────────────────────────
    //  Aim PIDF constants
    //  Active in AIMING, SPINNING, and FIRING states.
    //  In FIRING, headingOffsetRad from velocity comp is added to the target.
    // ─────────────────────────────────────────────
    public static double Kp_aim           = 0.6;
    public static double Ki_aim           = 0.0;
    public static double Kd_aim           = 0.002;
    public static double Kf_aim           = 0.2;
    public static double headingTolerance = 0.01; // ~1.7°

    public static double AIM_MAX_YAW_LONG_RANGE   = 1.0;
    public static double LONG_RANGE_AIM_THRESHOLD = 110.0;

    private PIDFController aimPid;

    // ─────────────────────────────────────────────
    //  Idle / manual shooter defaults
    // ─────────────────────────────────────────────
    public static double IDLE_RPM  = 1100;
    public static double IDLE_HOOD = 0.520;

    public static double MANUAL_RPM  = 1100;
    public static double MANUAL_HOOD = 0.48;

    public static double rpmTolerance = 50;

    // ─────────────────────────────────────────────
    //  Block servo
    // ─────────────────────────────────────────────
    private static final double BLOCK_SERVO_DOWN       = 0.8;
    private static final double BLOCK_SERVO_UP         = 0.25;
    public  static       double BLOCK_OPEN_DURATION_MS = 1000;

    public static double LONG_RANGE_INTAKE_SPEED = -0.85;

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
    //  Limelight / distance state
    // ─────────────────────────────────────────────
    private static final long TARGET_HOLD_MS          = 30;
    private static final long CAMERA_BLOCK_TIMEOUT_MS = 400;

    private double lastTx              = 0.0;
    private long   lastSeenTimeMs      = 0;
    private long   lastValidTargetTime = 0;

    private boolean cameraBlocked      = false;
    private boolean hasRumbledForBlock = false;

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

    // ─────────────────────────────────────────────
    //  Live shooter state
    //  Updated every loop in FIRING (physics), or once at trigger press (LUT/manual).
    // ─────────────────────────────────────────────
    private double liveTargetRPM  = IDLE_RPM;
    private double liveHoodTarget = IDLE_HOOD;

    // ─────────────────────────────────────────────
    //  Debug telemetry
    // ─────────────────────────────────────────────
    private double dbgLaunchAngleDeg  = 0;
    private double dbgLaunchSpeedIPS  = 0;
    private double dbgVrr             = 0;
    private double dbgVrt             = 0;
    private double dbgVxComp          = 0;
    private double dbgTurretOffsetDeg = 0;

    // =========================================================
    //  runOpMode
    // =========================================================
    @Override
    public void runOpMode() {
        initHardware();

        aimPid = new PIDFController(new PIDFCoefficients(Kp_aim, Ki_aim, Kd_aim, Kf_aim));
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            follower.update();

            handleButtons();
            updateCameraBlockStatus();

            double axial   = -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // ── Trigger: start shoot sequence ──
            if (gamepad1.right_trigger > 0.3 && autoState == AutoState.IDLE) {
                startShootSequence();
            }

            // ── Current distance (used by every active state) ──
            distance = clampDistance(getStableDistance());

            // ────────────────────────────────────────────────────────────
            //  AIMING
            //  Auto-yaw only. Driver keeps full axial/lateral.
            //  Transitions to SPINNING once heading is settled.
            // ────────────────────────────────────────────────────────────
            if (autoState == AutoState.AIMING) {
                Pose robotPose = follower.getPose();
                yaw = runAimPidf(robotPose, 0.0);

                if (Math.abs(getHeadingError(robotPose, computeTargetHeading(robotPose)))
                        < headingTolerance) {
                    autoState = AutoState.SPINNING;
                }
            }

            // ────────────────────────────────────────────────────────────
            //  SPINNING
            //  Hold heading while flywheel spins up. Driver keeps axial/lateral.
            //  Transitions to FIRING once atRPMTarget (handled in state machine).
            // ────────────────────────────────────────────────────────────
            if (autoState == AutoState.SPINNING) {
                yaw = runAimPidf(follower.getPose(), 0.0);
            }

            // ────────────────────────────────────────────────────────────
            //  FIRING
            //  Live velocity compensation every loop:
            //    - Radial Vrr  → recalculate RPM + hood in real time
            //    - Tangential Vrt → shift heading target (section B7 of PDF)
            //  Yaw fully overridden. Driver keeps axial/lateral.
            // ────────────────────────────────────────────────────────────
            if (autoState == AutoState.FIRING) {
                if (USE_PHYSICS_CALC) {
                    double[] vel = getRobotVelocityComponents(follower.getPose());
                    double vrr   = vel[0]; // radial  (in/s)
                    double vrt   = vel[1]; // tangential (in/s)
                    dbgVrr = vrr;
                    dbgVrt = vrt;

                    // Full velocity-compensated physics solve (PDF sections A + B)
                    double[] shot          = solvePhysicsWithVelocity(distance, vrr, vrt);
                    liveTargetRPM          = shot[0];
                    liveHoodTarget         = shot[1];
                    double headingOffsetRad = shot[2]; // tangential correction angle

                    // Aim PIDF with tangential heading offset applied
                    yaw = runAimPidf(follower.getPose(), headingOffsetRad);
                } else {
                    // Legacy: just hold heading, no live compensation
                    yaw = runAimPidf(follower.getPose(), 0.0);
                }
            }

            driveMecanum(axial, lateral, yaw);

            // ── Final RPM / hood resolution ──
            double targetRPM;
            double hoodTarget;
            if (autoState == AutoState.IDLE) {
                targetRPM  = IDLE_RPM;
                hoodTarget = IDLE_HOOD;
            } else {
                // AIMING, SPINNING, FIRING all use liveTargetRPM / liveHoodTarget.
                // In FIRING+physics these are updated every loop above;
                // in AIMING/SPINNING they hold the snapshot from trigger press.
                targetRPM  = liveTargetRPM;
                hoodTarget = liveHoodTarget;
            }

            applyShooterPower(targetRPM, hoodTarget);
            runStateMachine(targetRPM);
            handleManualBlock();
            updateTelemetry(targetRPM, hoodTarget, yaw);
        }
    }

    // =========================================================
    //  Physics  — Section A  (PDF pages 1–4)
    // =========================================================

    /**
     * Launch angle α for a stationary robot.
     *
     * PDF p.3 step 5:
     *   α = arctan( 2y/x − tan(θ) )
     *
     * x = horizontal distance, y = GOAL_HEIGHT_IN, θ = ENTRY_ANGLE_DEG
     */
    private double calcLaunchAngleDeg(double x) {
        double y        = GOAL_HEIGHT_IN;
        double theta    = Math.toRadians(ENTRY_ANGLE_DEG);
        double tanAlpha = (2.0 * y / x) - Math.tan(theta);
        return Math.toDegrees(Math.atan(tanAlpha));
    }

    /**
     * Initial launch speed v0.
     *
     * PDF p.4 step 6:
     *   v0 = sqrt( g·x² / (2·cos²(α)·(x·tan(α) − y)) )
     */
    private double calcLaunchSpeedIPS(double x, double alphaDeg) {
        double alpha = Math.toRadians(alphaDeg);
        double g     = GRAVITY_IN_S2;
        double cosA  = Math.cos(alpha);
        double denom = 2.0 * cosA * cosA * (x * Math.tan(alpha) - GOAL_HEIGHT_IN);
        if (denom <= 0) return 0;
        return Math.sqrt((g * x * x) / denom);
    }

    // =========================================================
    //  Physics  — Section B  (PDF pages 4–6)
    // =========================================================

    /**
     * Decompose robot velocity into radial and tangential components
     * relative to the robot→goal line.
     *
     * PDF p.4 section B step 1:
     *   θ_diff = θ_velocity − θ_line
     *   Vrr (radial)     = −cos(θ_diff) · Vmag   (negative = moving toward goal)
     *   Vrt (tangential) =  sin(θ_diff) · Vmag
     *
     * @return double[]{ Vrr (in/s), Vrt (in/s) }
     */
    private double[] getRobotVelocityComponents(Pose robotPose) {
        Vector vel   = follower.getVelocity();
        double vx    = vel.getXComponent() * PEDRO_VEL_TO_IPS;
        double vy    = vel.getYComponent() * PEDRO_VEL_TO_IPS;
        double vmag  = Math.hypot(vx, vy);

        double thetaVel  = Math.atan2(vy, vx);
        double thetaLine = Math.atan2(
                SHOOT_TARGET_Y - robotPose.getY(),
                SHOOT_TARGET_X - robotPose.getX());

        double theta = thetaVel - thetaLine;

        return new double[]{
                -Math.cos(theta) * vmag,   // Vrr
                Math.sin(theta) * vmag    // Vrt
        };
    }

    /**
     * Full velocity-compensated physics solve.
     *
     * PDF section B steps 2–7 + section C:
     *
     *   B2. t = x / (v0·cos(α))
     *   B3. Vx_comp = x/t + Vrr
     *       Vx_new  = sqrt(Vx_comp² + Vrt²)
     *   B4. Vy = v0·sin(α)
     *   B5. α_new = arctan(Vy / Vx_new)  [clamped to hood limits]
     *   B6. x_new = Vx_new · t
     *       v0_new = calcLaunchSpeedIPS(x_new, α_new)
     *   B7. Heading offset = atan(Vrt / Vx_comp)
     *   C.  RPM = launchSpeedToRPM(v0_new)
     *       servo = angleToServoPosition(α_new)
     *
     * @return double[]{ targetRPM, hoodServoPos, headingOffsetRad }
     */
    private double[] solvePhysicsWithVelocity(double distanceIn, double vrr, double vrt) {
        // Baseline stationary solve
        double alphaDeg        = calcLaunchAngleDeg(distanceIn);
        double clampedAlphaDeg = clamp(alphaDeg, HOOD_MIN_ANGLE_DEG, HOOD_MAX_ANGLE_DEG);
        double v0              = calcLaunchSpeedIPS(distanceIn, clampedAlphaDeg);

        if (v0 <= 0 || Double.isNaN(v0)) {
            return new double[]{ IDLE_RPM, IDLE_HOOD, 0.0 };
        }

        double alpha = Math.toRadians(clampedAlphaDeg);

        // B2 — time of flight
        double t = distanceIn / (v0 * Math.cos(alpha));

        // B3 — compensated horizontal speed
        double vxComp = (distanceIn / t) + vrr;
        double vxNew  = Math.hypot(vxComp, vrt);

        // B4 — vertical component unchanged
        double vy = v0 * Math.sin(alpha);

        // B5 — new launch angle
        double alphaDegNew        = Math.toDegrees(Math.atan2(vy, vxNew));
        double clampedAlphaDegNew = clamp(alphaDegNew, HOOD_MIN_ANGLE_DEG, HOOD_MAX_ANGLE_DEG);

        // B6 — new launch speed
        double xNew  = vxNew * t;
        double v0New = calcLaunchSpeedIPS(xNew, clampedAlphaDegNew);
        if (v0New <= 0 || Double.isNaN(v0New)) v0New = v0; // fallback

        // B7 — heading offset for tangential velocity
        double headingOffsetRad = Math.atan2(vrt, vxComp);

        // Section C — convert to hardware commands
        double rpm      = launchSpeedToRPM(v0New);
        double servoPos = angleToServoPosition(clampedAlphaDegNew);

        // Debug
        dbgLaunchAngleDeg  = clampedAlphaDegNew;
        dbgLaunchSpeedIPS  = v0New;
        dbgVxComp          = vxComp;
        dbgTurretOffsetDeg = Math.toDegrees(headingOffsetRad);

        return new double[]{ rpm, servoPos, headingOffsetRad };
    }

    /**
     * Stationary physics solve — used at trigger press to prime RPM/hood
     * before AIMING completes (no velocity data needed yet).
     */
    private double[] solvePhysics(double distanceIn) {
        double alphaDeg        = calcLaunchAngleDeg(distanceIn);
        double clampedAlphaDeg = clamp(alphaDeg, HOOD_MIN_ANGLE_DEG, HOOD_MAX_ANGLE_DEG);
        double v0IPS           = calcLaunchSpeedIPS(distanceIn, clampedAlphaDeg);

        dbgLaunchAngleDeg = clampedAlphaDeg;
        dbgLaunchSpeedIPS = v0IPS;

        if (v0IPS <= 0 || Double.isNaN(v0IPS)) {
            return new double[]{ IDLE_RPM, IDLE_HOOD };
        }
        return new double[]{ launchSpeedToRPM(v0IPS), angleToServoPosition(clampedAlphaDeg) };
    }

    // =========================================================
    //  Physics  — Section C  (PDF pages 6–7)
    // =========================================================

    /**
     * Convert physical launch angle (degrees) to servo position.
     *
     * PDF p.6:  Servo Position = ((s1−s2)/(a1−a2)) · (α−a1) + s1
     */
    private double angleToServoPosition(double angleDeg) {
        double a1 = HOOD_ANGLE_1_DEG, a2 = HOOD_ANGLE_2_DEG;
        double s1 = HOOD_SERVO_POS_1,  s2 = HOOD_SERVO_POS_2;
        double clamped  = clamp(angleDeg, HOOD_MIN_ANGLE_DEG, HOOD_MAX_ANGLE_DEG);
        double servoPos = ((s1 - s2) / (a1 - a2)) * (clamped - a1) + s1;
        return clamp(servoPos, Math.min(s1, s2), Math.max(s1, s2));
    }

    /**
     * Convert launch speed (inches/s) to flywheel RPM.
     *
     * Linear fit:  RPM = RPM_SLOPE · v0 + RPM_INTERCEPT
     */
    private double launchSpeedToRPM(double v0IPS) {
        return RPM_SLOPE * v0IPS + RPM_INTERCEPT;
    }

    // =========================================================
    //  Aim PIDF
    //  headingOffsetRad: tangential correction from section B7 (0 when AIMING/SPINNING).
    // =========================================================
    private double runAimPidf(Pose robotPose, double headingOffsetRad) {
        double targetHeading = computeTargetHeading(robotPose) + headingOffsetRad;
        double headingError  = getHeadingError(robotPose, targetHeading);

        aimPid.setCoefficients(new PIDFCoefficients(Kp_aim, Ki_aim, Kd_aim, Kf_aim));
        aimPid.updateError(headingError);
        aimPid.updateFeedForwardInput(Math.signum(headingError));

        double yawLimit = (distance > LONG_RANGE_AIM_THRESHOLD) ? AIM_MAX_YAW_LONG_RANGE : 1.0;
        return clamp(-aimPid.run(), -yawLimit, yawLimit);
    }

    private double getHeadingError(Pose robotPose, double targetHeading) {
        return MathFunctions.getTurnDirection(robotPose.getHeading(), targetHeading)
                * MathFunctions.getSmallestAngleDifference(robotPose.getHeading(), targetHeading);
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
                limelight.start();
                gamepad1.setLedColor(0, 0, 1, LED_DURATION_CONTINUOUS);
            }
        }

        if (gamepad1.dpadDownWasPressed()) relocalize();

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
    private void startShootSequence() {
        preShootIntakeState = intakeOn;
        intakeOn = 0;
        intakeMotor.setPower(0);

        if (manualOverride) {
            liveTargetRPM  = MANUAL_RPM;
            liveHoodTarget = MANUAL_HOOD;
            autoState      = AutoState.SPINNING; // manual skips aiming
        } else {
            distance = clampDistance(getStableDistance());

            // Prime a stationary snapshot — will be updated live once FIRING begins
            if (USE_PHYSICS_CALC) {
                double[] shot  = solvePhysics(distance);
                liveTargetRPM  = shot[0];
                liveHoodTarget = shot[1];
            } else {
                liveTargetRPM  = legacyRPM(distance);
                liveHoodTarget = legacyHood(distance);
            }

            aimPid             = new PIDFController(new PIDFCoefficients(Kp_aim, Ki_aim, Kd_aim, Kf_aim));
            autoState          = AutoState.AIMING;
            hasRumbledForBlock = false;
        }
    }

    // =========================================================
    //  Legacy LUT (kept for A/B comparison)
    // =========================================================
    private double legacyRPM(double d) {
        if (d <= 25)  return 1000;
        if (d <= 30)  return 1000;
        if (d <= 35)  return 1050;
        if (d <= 40)  return 1050;
        if (d <= 45)  return 1100;
        if (d <= 50)  return 1140;
        if (d <= 55)  return 1150;
        if (d <= 65)  return 1150;
        if (d <= 70)  return 1250;
        if (d <= 75)  return 1250;
        if (d <= 85)  return 1280;
        if (d <= 115) return 1320;
        if (d <= 120) return 1330;
        if (d <= 125) return 1350;
        if (d <= 135) return 1350;
        return 1360;
    }

    private double legacyHood(double d) {
        if (d <= 35)  return 0.482;
        if (d <= 45)  return 0.480;
        if (d <= 50)  return 0.482;
        if (d <= 55)  return 0.486;
        if (d <= 60)  return 0.486;
        if (d <= 65)  return 0.495;
        if (d <= 75)  return 0.497;
        if (d <= 85)  return 0.500;
        if (d <= 115) return 0.539;
        if (d <= 120) return 0.540;
        if (d <= 125) return 0.542;
        return 0.542;
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

    private double computeTargetHeading(Pose robotPose) {
        return Math.atan2(
                SHOOT_TARGET_Y - robotPose.getY(),
                SHOOT_TARGET_X - robotPose.getX());
    }

    // =========================================================
    //  Relocalization
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
        if (result != null && result.isValid()) {
            Pose3D robotPos = result.getBotpose();
            double angle    = robotPos.getOrientation().getYaw(AngleUnit.DEGREES) - 90;
            if (angle > 360) angle -= 360;
            return new Pose(
                    robotPos.getPosition().y / 0.0254 + 70.625,
                    -robotPos.getPosition().x / 0.0254 + 70.625,
                    Math.toRadians(angle));
        }
        return null;
    }

    // =========================================================
    //  PV Shooter controller
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
    //  Auto-shoot state machine
    // =========================================================
    private void runStateMachine(double targetRPM) {
        switch (autoState) {

            case SPINNING:
                if (atRPMTarget(targetRPM)) {
                    autoState = AutoState.FIRING;
                }
                break;

            case FIRING:
                // Open block servo on first entry
                if (!isBlocking) {
                    isBlocking = true;
                    blockTimer.reset();
                    blockServoUpTimer.reset();
                    intakeStarted = false;
                    blockServo.setPosition(BLOCK_SERVO_UP);
                }

                // Start intake after delay
                if (!intakeStarted && blockServoUpTimer.milliseconds() >= BLOCK_INTAKE_DELAY_MS) {
                    intakeStarted = true;
                    intakeMotor.setPower(distance > LONG_RANGE_AIM_THRESHOLD
                            ? LONG_RANGE_INTAKE_SPEED : -1.0);
                    intakeOn = 1;
                }

                // End of firing window
                if (blockTimer.milliseconds() >= BLOCK_OPEN_DURATION_MS) {
                    blockServo.setPosition(BLOCK_SERVO_DOWN);
                    isBlocking    = false;
                    intakeStarted = false;
                    autoState     = AutoState.IDLE;
                    intakeOn      = preShootIntakeState;
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
        autoState  = AutoState.IDLE;
        isBlocking = false;
        blockServo.setPosition(BLOCK_SERVO_DOWN);
        telemetry.addData("Abort", reason);
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
        Pose robotPose = follower.getPose();
        return Math.hypot(
                SHOOT_TARGET_X - robotPose.getX(),
                SHOOT_TARGET_Y - robotPose.getY());
    }

    private double clampDistance(double d) {
        if (d < 22)   return 23;
        if (d <= 85)  return d;
        if (d < 110)  return 79;
        if (d <= 140) return d;
        return 139;
    }

    // =========================================================
    //  Telemetry
    // =========================================================
    private void updateTelemetry(double targetRPM, double hoodTarget, double yaw) {
        double actualRPM     = shootMotor.getVelocity();
        double rpmError      = targetRPM - actualRPM;
        Pose   robotPose     = follower.getPose();
        double targetHeading = computeTargetHeading(robotPose);
        double headingError  = getHeadingError(robotPose, targetHeading);

        telemetry.addData("State/AutoState",         autoState.toString());
        telemetry.addData("State/ManualOverride",     manualOverride);
        telemetry.addData("State/ShooterOFF",         shooterOverride);
        telemetry.addData("State/CameraBlocked",      cameraBlocked);
        telemetry.addData("State/UsePhysicsCalc",     USE_PHYSICS_CALC);

        telemetry.addData("Shooter/TargetRPM",        targetRPM);
        telemetry.addData("Shooter/ActualRPM",        actualRPM);
        telemetry.addData("Shooter/RPM_Error",        rpmError);
        telemetry.addData("Shooter/AtTarget",         atRPMTarget(targetRPM));
        telemetry.addData("Shooter/ShootPower",       shootMotor.getPower());
        telemetry.addData("Shooter/LiftRPM",          shootMotor2.getVelocity());

        telemetry.addData("Hood/Target",              hoodTarget);
        telemetry.addData("Hood/Position",            hoodServo.getPosition());

        telemetry.addData("Physics/LaunchAngle_deg",  dbgLaunchAngleDeg);
        telemetry.addData("Physics/LaunchSpeed_ips",  dbgLaunchSpeedIPS);
        telemetry.addData("Physics/GoalHeight_in",    GOAL_HEIGHT_IN);
        telemetry.addData("Physics/EntryAngle_deg",   ENTRY_ANGLE_DEG);
        telemetry.addData("Physics/HoodMin_deg",      HOOD_MIN_ANGLE_DEG);
        telemetry.addData("Physics/HoodMax_deg",      HOOD_MAX_ANGLE_DEG);
        telemetry.addData("Physics/RPM_Slope",        RPM_SLOPE);
        telemetry.addData("Physics/RPM_Intercept",    RPM_INTERCEPT);

        telemetry.addData("VelComp/Vrr_ips",          dbgVrr);
        telemetry.addData("VelComp/Vrt_ips",          dbgVrt);
        telemetry.addData("VelComp/Vx_compensated",   dbgVxComp);
        telemetry.addData("VelComp/TurretOffset_deg", dbgTurretOffsetDeg);
        telemetry.addData("VelComp/PedroVelScale",    PEDRO_VEL_TO_IPS);

        telemetry.addData("PV/kS",                    kS);
        telemetry.addData("PV/kV",                    kV);
        telemetry.addData("PV/kP",                    kP);
        telemetry.addData("PV/term_kV",               kV * targetRPM);
        telemetry.addData("PV/term_kP",               kP * rpmError);

        telemetry.addData("Aim/TargetHeading_deg",    Math.toDegrees(targetHeading));
        telemetry.addData("Aim/CurrentHeading_deg",   Math.toDegrees(robotPose.getHeading()));
        telemetry.addData("Aim/HeadingError_deg",     Math.toDegrees(headingError));
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

        telemetry.addData("Distance/Raw_in",          distance);
        telemetry.addData("Distance/Clamped_in",      clampDistance(distance));

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
