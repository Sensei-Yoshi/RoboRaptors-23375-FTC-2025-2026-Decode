package org.firstinspires.ftc.teamcode.Testing;

import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.List;

@TeleOp(name="LimelightShotTuning", group="Tuning")
public class LimelightTuning extends LinearOpMode {
    private ElapsedTime pushTimer1 = new ElapsedTime();

    DcMotorEx shootMotor, intakeMotor;
    Servo hoodServo, pushServo, blockServo;

    double shooterVelocity = 1050;
    double hoodPos = 0.40;



    Limelight3A limelight;
    ElapsedTime pushTimer = new ElapsedTime();
    boolean isPushing = false;
    boolean isPushingManual = false;
    int intakeOn = 0;
    int shots = 0;
    final double pushServoDown = 0.9; //change if too close to ground: <0.9 == up and >0.9 = down
    final double pushServoUp = 0.5;

    final double blockServoDown = 0.84; //if two balls are shooting at once: <0.81 == up and >0.81 == down
    final double blockServoUp = 0.25;

    @Override
    public void runOpMode() {

        shootMotor = hardwareMap.get(DcMotorEx.class, "shootMotor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        pushServo = hardwareMap.get(Servo.class, "pushServo");
        blockServo = hardwareMap.get(Servo.class, "blockServo");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.start();
        limelight.pipelineSwitch(0);

        shootMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shootMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        shootMotor.setVelocity(shooterVelocity);
        hoodServo.setPosition(hoodPos);
        pushServo.setPosition(pushServoDown);
        blockServo.setPosition(blockServoDown);

        waitForStart();

        while (opModeIsActive()) {

            // ───── Tuning controls ─────
            if (gamepad1.dpadUpWasPressed()) shooterVelocity += 10;
            if (gamepad1.dpadDownWasPressed()) shooterVelocity -= 10;
            if (gamepad1.dpadRightWasPressed()) hoodPos += 0.002;
            if (gamepad1.dpadLeftWasPressed()) hoodPos -= 0.002;

            shooterVelocity = Math.max(0, shooterVelocity);
            hoodPos = Math.min(1.0, Math.max(0.0, hoodPos));

            shootMotor.setVelocity(shooterVelocity);
            hoodServo.setPosition(hoodPos);

            // ───── Intake ─────
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
            }
            else if (intakeOn == 2) {
                intakeMotor.setPower(1);
            }
            else {
                intakeMotor.setPower(0);
            }

            // ───── Rapid fire ─────
            //Rapid Shooting: Starts
            if (gamepad1.right_trigger > 0.3 && !isPushing && shots == 0) {
                shots = 3;
                blockServo.setPosition(blockServoUp);
            }
            if (!isPushing && shots > 0) {
                isPushing = true;
                pushTimer1.reset();
                pushServo.setPosition(pushServoUp);     // PUSH UP
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
            //Ends

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

            double distance = distanceFromRed();

            telemetry.addLine("==== SHOOTER TUNING ====");
            telemetry.addData("Distance (in)", "%.1f", distance);
            telemetry.addData("Target Velocity", "%.0f", shooterVelocity);
            telemetry.addData("Velocity", "%.0f", shootMotor.getVelocity());
            telemetry.addData("Hood", "%.3f", hoodPos);
            telemetry.addData("Shots queued", shots);
            telemetry.update();
        }
    }

    // ───── Distance Functions ─────
    public double distanceFromTag(double tagID) {
        List<LLResultTypes.FiducialResult> r = limelight.getLatestResult().getFiducialResults();
        if (r.isEmpty()) return 0;

        for (LLResultTypes.FiducialResult i : r) {
            if (i != null && i.getFiducialId() == tagID) {
                double x = i.getCameraPoseTargetSpace().getPosition().x / DistanceUnit.mPerInch;
                double z = i.getCameraPoseTargetSpace().getPosition().z / DistanceUnit.mPerInch;
                Vector e = new Vector();
                e.setOrthogonalComponents(x, z);
                return e.getMagnitude();
            }
        }
        return 0;
    }

    public double distanceFromRed() {
        return distanceFromTag(24);
    }
}