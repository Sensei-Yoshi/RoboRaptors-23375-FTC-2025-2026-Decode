package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.Gamepad.LED_DURATION_CONTINUOUS;

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
@TeleOp(name="Red TeleOp", group="Tuning")
public class NewTeleOp extends LinearOpMode {

    private enum AutoState { IDLE, AIMING, SPINNING, FIRING }
    private com.arcrobotics.ftclib.controller.PIDFController aimPid;
    private AutoState autoState = AutoState.IDLE;
    ElapsedTime fireDelayTimer = new ElapsedTime();
    boolean fireDelayActive = false;

    DcMotorEx shootMotor, intakeMotor;
    Servo hoodServo, blockServo, light;
    InterpLUT controlPointsRPM = new InterpLUT();
    InterpLUT controlPointsHood = new InterpLUT();
    DcMotor leftFrontDrive = null;
    DcMotor leftBackDrive = null;
    DcMotor rightFrontDrive = null;
    DcMotor rightBackDrive = null;
    private DcMotorEx liftMotor = null;
    private Servo pushServo = null;


    double hoodPos = 0.40;

    Limelight3A limelight;
    ElapsedTime blockTimer = new ElapsedTime();
    boolean isBlocking = false;
    boolean autoAimActive = false;
    boolean shooterReady = false;
    boolean aimReady = false;

    int intakeOn = 0;
    final double blockServoDown = 0.84; //if two balls are shooting at once: <0.81 == up and >0.81 == down
    final double blockServoUp = 0.25;

    public static double Kp = 0.02;
    public static double Ki = 0.0;
    public static double Kd = 0.003;
    public static double Kf = 0.18;
    double targetPose = 0;
    public static double rpmTolerance = 150;
    public static double aimTolerance = 1.5;
    double distance;
    ElapsedTime blockDelayTimer = new ElapsedTime();
    boolean blockDelayActive = false;
    final double BLOCK_OPEN_DELAY = 25;
    private double lastTx = 0.0;
    private long lastSeenTimeMs = 0;
    private static final long TARGET_HOLD_MS = 30;

    private double lastDistance = 0.0;
    private long lastDistanceSeenTimeMs = 0;

    private boolean velocityLocked = false;
    private double lockedDistance = 0.0;
    private double lockedRPM = 0.0;
    private double lockedHood = 0.0;
    boolean shooterOverride = false;
    double lastAutoRPM = 0;
    private boolean cameraBlocked = false;
    private long lastValidTargetTime = 0;
    private boolean hasRumbledForBlock = false;
    private static final long CAMERA_BLOCK_TIMEOUT_MS = 400;

    private boolean manualOverride = false;
    private static final double MANUAL_RPM = 1100;
    private static final double MANUAL_HOOD = 0.48;

    // Idle (moderate) speed the shooter holds when not firing.
    // Roughly the midpoint of the RPM and hood LUTs.
    public static double IDLE_RPM = 1200;
    public static double IDLE_HOOD = 0.520;
    final double pushServoDown = 0.9;

    // Block servo open duration for shooting
    public static double BLOCK_OPEN_DURATION_MS = 1000;



    @Override
    public void runOpMode() {

        shootMotor = hardwareMap.get(DcMotorEx.class, "shootMotor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        blockServo = hardwareMap.get(Servo.class, "blockServo");
        pushServo = hardwareMap.get(Servo.class, "pushServo");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        light = hardwareMap.get(Servo.class, "light");
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
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
        liftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        liftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new com.qualcomm.robotcore.hardware.PIDFCoefficients(300, 0, 0, 10));
        liftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        hoodServo.setPosition(hoodPos);
        blockServo.setPosition(blockServoDown);
        pushServo.setPosition(pushServoDown);

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
            if (gamepad1.aWasPressed()) {
                if (!shooterOverride) {
                    // Turn shooter OFF
                    shootMotor.setVelocity(0);
                    liftMotor.setVelocity(0);
                    shooterOverride = true;
                    gamepad1.setLedColor(0, 1, 0, LED_DURATION_CONTINUOUS); // green
                } else {
                    // Turn shooter back ON to auto mode
                    shooterOverride = false;
                    gamepad1.setLedColor(0, 0, 1, LED_DURATION_CONTINUOUS); // blue
                }
            }

            if (gamepad1.dpadUpWasPressed()) {
                if (!manualOverride) {
                    // Turn ON manual override
                    manualOverride = true;
                    gamepad1.setLedColor(1, 1, 0, LED_DURATION_CONTINUOUS); // yellow
                } else {
                    // Turn OFF manual override
                    manualOverride = false;
                    limelight.start();
                    gamepad1.setLedColor(0, 0, 1, LED_DURATION_CONTINUOUS); // blue
                }
            }

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
            updateCameraBlockStatus();

            // ── Lock velocity/hood the instant the shoot button is pressed ──
            if (gamepad1.right_trigger > 0.3 && autoState == AutoState.IDLE) {
                if (manualOverride) {
                    velocityLocked = true;
                    lockedRPM = MANUAL_RPM;
                    lockedHood = MANUAL_HOOD;
                    autoState = AutoState.SPINNING;
                } else if (cameraBlocked) {
                    gamepad1.rumble(1.0, 1.0, 500);
                    hasRumbledForBlock = true;
                } else {
                    // Snapshot current distance-based values and lock them
                    distance = clampDistance(getStableDistance());
                    velocityLocked = true;
                    lockedDistance = distance;
                    lockedRPM = controlPointsRPM.get(distance);
                    lockedHood = controlPointsHood.get(distance);
                    autoState = AutoState.AIMING;
                    hasRumbledForBlock = false;
                }
            }

            if (autoState == AutoState.AIMING) {
                // Check if camera becomes blocked during aiming
                if (cameraBlocked) {
                    if (!hasRumbledForBlock) {
                        gamepad1.rumble(1.0, 1.0, 500);
                        hasRumbledForBlock = true;
                    }
                    // Abort auto aim sequence; unlock so shooter goes back to live tracking
                    autoState = AutoState.IDLE;
                    velocityLocked = false;
                    blockDelayActive = false;
                } else {
                    double error = getTx(24);
                    yaw = (-aimPid.calculate(error) + (Kf * Math.signum(error)));
                    if (aimPid.atSetPoint()) {
                        autoState = AutoState.SPINNING;
                        axial = 0;
                        lateral = 0;
                    }
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


            // ── Determine what RPM/hood to command ──
            // Default: idle RPM/hood.
            // Only switch to distance-based (locked) values while a shoot sequence is active.
            double targetRPM;
            double hoodTarget;

            distance = clampDistance(getStableDistance());

            if (velocityLocked) {
                // Active shoot sequence: hold the locked distance-based values
                targetRPM = lockedRPM;
                hoodTarget = lockedHood;
            } else {
                // Not firing: run at idle speed
                targetRPM = IDLE_RPM;
                hoodTarget = IDLE_HOOD;
            }

            if (manualOverride) {
                // Manual override always uses its fixed values
                shootMotor.setVelocity(MANUAL_RPM);
                liftMotor.setVelocity(MANUAL_RPM);
                hoodServo.setPosition(MANUAL_HOOD);
            } else if (!shooterOverride) {
                shootMotor.setVelocity(targetRPM);
                liftMotor.setVelocity(targetRPM);
                hoodServo.setPosition(hoodTarget);
                lastAutoRPM = targetRPM;
            } else {
                // Shooter override - OFF
                shootMotor.setVelocity(0);
                liftMotor.setVelocity(0);
            }

            switch (autoState) {

                case SPINNING:
                    if (cameraBlocked && !manualOverride) {
                        if (!hasRumbledForBlock) {
                            gamepad1.rumble(1.0, 1.0, 500);
                            hasRumbledForBlock = true;
                        }
                        autoState = AutoState.IDLE;
                        velocityLocked = false;
                        blockDelayActive = false;
                    } else if (Math.abs(shootMotor.getVelocity() - targetRPM) < rpmTolerance) {
                        autoState = AutoState.FIRING;
                    }
                    break;
                case FIRING:
                    if (cameraBlocked && !manualOverride) {
                        if (!hasRumbledForBlock) {
                            gamepad1.rumble(1.0, 1.0, 500);
                            hasRumbledForBlock = true;
                        }
                        // Abort firing, unlock so shooter goes back to live tracking
                        autoState = AutoState.IDLE;
                        firingArmed = false;
                        velocityLocked = false;
                        blockDelayActive = false;
                        isBlocking = false;
                        blockServo.setPosition(blockServoDown);
                        break;
                    }

                    // Start the blocking sequence
                    if (!isBlocking) {
                        isBlocking = true;
                        blockTimer.reset();
                        blockServo.setPosition(blockServoUp);
                    }

                    // Check if block duration has elapsed
                    if (blockTimer.milliseconds() >= BLOCK_OPEN_DURATION_MS) {
                        // Close block servo and end firing sequence
                        blockServo.setPosition(blockServoDown);
                        isBlocking = false;
                        velocityLocked = false;  // Unlock — shooter returns to idle
                        autoState = AutoState.IDLE;
                    }
                    break;
            }

            // Manual block servo control with right bumper
            if (gamepad1.rightBumperWasPressed() && !isBlocking) {
                isBlocking = true;
                blockTimer.reset();
                blockServo.setPosition(blockServoUp);
            }

            if (isBlocking && autoState == AutoState.IDLE) {
                // Manual block servo control (only when not in auto firing)
                if (blockTimer.milliseconds() >= BLOCK_OPEN_DURATION_MS) {
                    blockServo.setPosition(blockServoDown);
                    isBlocking = false;
                }
            }


            telemetry.addData("Kp", aimPid.getP());
            telemetry.addData("KF", aimPid.getF());
            telemetry.addData("Kd", aimPid.getD());
            telemetry.addData("State", autoState);
            telemetry.addData("Manual Override", manualOverride);
            telemetry.addData("Camera Blocked", cameraBlocked);
            telemetry.addData("Yaw", yaw);
            telemetry.addData("Distance (in)", "%.1f", distance);
            telemetry.addData("Idle RPM", "%.0f", IDLE_RPM);
            telemetry.addData("Idle Hood", "%.3f", IDLE_HOOD);
            telemetry.addData("Locked RPM", "%.0f", lockedRPM);
            telemetry.addData("Velocity", "%.0f", shootMotor.getVelocity());
            telemetry.addData("Hood", "%.3f", hoodServo.getPosition());
            telemetry.addData("TX", "%.3f", getTx(24));
            telemetry.addData("Block Timer", "%.0f ms", blockTimer.milliseconds());
            telemetry.update();
        }
    }


    // ───── Distance Functions ─────
    private double distanceFromTag(double tagID) {
        List<LLResultTypes.FiducialResult> r = limelight.getLatestResult().getFiducialResults();
        if (r.isEmpty()) {
            light.setPosition(0.388);
            return 0.0;

        }

        for (LLResultTypes.FiducialResult i : r) {
            if (i != null && i.getFiducialId() == tagID) {
                light.setPosition(0.728);
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
        if (distance <= 85) return distance;

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
        controlPointsRPM.add(50, 1280);
        controlPointsRPM.add(55, 1280);
        controlPointsRPM.add(60, 1280);
        controlPointsRPM.add(65, 1280);
        controlPointsRPM.add(70, 1400);
        controlPointsRPM.add(75, 1400);
        controlPointsRPM.add(80, 1500);
        controlPointsRPM.add(85, 1500);
        controlPointsRPM.add(110, 1640);
        controlPointsRPM.add(115, 1640);
        controlPointsRPM.add(120, 1650);
        controlPointsRPM.add(125, 1650);
        controlPointsRPM.add(130, 1660);
        controlPointsRPM.add(135, 1710);
        controlPointsRPM.createLUT();
    }


    public void createHoodControlPoints() {
        controlPointsHood.add(22, 0.482);
        controlPointsHood.add(25, 0.482);
        controlPointsHood.add(30, 0.482);
        controlPointsHood.add(35, 0.482);
        controlPointsHood.add(40, 0.482);
        controlPointsHood.add(45, 0.482);
        controlPointsHood.add(50, 0.482);
        controlPointsHood.add(55, 0.486);
        controlPointsHood.add(60, 0.486);
        controlPointsHood.add(65, 0.495);
        controlPointsHood.add(70, 0.498);
        controlPointsHood.add(75, 0.498);
        controlPointsHood.add(80, 0.510);
        controlPointsHood.add(85, 0.510);
        controlPointsHood.add(110, 0.522);
        controlPointsHood.add(115, 0.522);
        controlPointsHood.add(120, 0.524);
        controlPointsHood.add(125, 0.526);
        controlPointsHood.add(130, 0.526);
        controlPointsHood.add(135, 0.526);
        controlPointsHood.createLUT();
    }
    private double getStableDistance() {
        double d = distanceFromRed();
        long now = System.currentTimeMillis();

        if (d > 0) {
            lastDistance = d;
            lastDistanceSeenTimeMs = now;
            return d;
        }

        // Hold last known distance briefly
        if (now - lastDistanceSeenTimeMs <= TARGET_HOLD_MS) {
            return lastDistance;
        }

        return 0;
    }

    private double getTx(double targetID) {
        LLResult result = limelight.getLatestResult();
        long now = System.currentTimeMillis();
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            if (fiducial != null && fiducial.getFiducialId() == targetID) {
                lastSeenTimeMs = now;
                lastValidTargetTime = now;
                cameraBlocked = false;
                lastTx = fiducial.getTargetXDegrees();
                return lastTx;
            }
        }


        if (now - lastSeenTimeMs <= TARGET_HOLD_MS) {
            return lastTx;
        }
        return 0;
    }
    private void updateCameraBlockStatus() {
        long now = System.currentTimeMillis();

        // If we have a valid target recently, camera is not blocked
        if (now - lastValidTargetTime <= CAMERA_BLOCK_TIMEOUT_MS) {
            cameraBlocked = false;
            hasRumbledForBlock = false;
        } else {
            // No valid target for CAMERA_BLOCK_TIMEOUT_MS - camera is blocked
            cameraBlocked = true;
        }
    }
}