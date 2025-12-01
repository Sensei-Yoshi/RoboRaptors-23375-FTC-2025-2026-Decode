package org.firstinspires.ftc.teamcode;


import static com.qualcomm.robotcore.hardware.Gamepad.LED_DURATION_CONTINUOUS;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "Linear Opmode")
public class TeleOp extends LinearOpMode {


    private ElapsedTime pushTimer1 = new ElapsedTime();
    private ElapsedTime pushTimer = new ElapsedTime();

    //Drive
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx intakeMotor = null;
    private DcMotorEx shootMotor = null;
    private Servo pushServo = null;
    private Servo blockServo = null;
    final double closeLaunch = 1050; //in ticks/second for the close goal.
    final double farLaunch = 1350;
    final double pushServoDown = 0.88;
    final double pushServoUp = 0.3;
    final double blockServoDown = 0.83;
    final double blockServoUp = 0.3;

    @Override
    public void runOpMode() {

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        shootMotor = hardwareMap.get(DcMotorEx.class, "shootMotor");
        pushServo = hardwareMap.get(Servo.class, "pushServo");
        blockServo = hardwareMap.get(Servo.class, "blockServo");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        shootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));


        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        int intakeOn = 0, shots = 0;
        pushServo.setPosition(pushServoDown);
        blockServo.setPosition(blockServoDown);
        waitForStart();
        boolean isPushingManual = false, isPushing = false, shootingState = false, flyWheelOn = true;
        shootMotor.setVelocity(closeLaunch);
        gamepad1.setLedColor(1, 0, 0, LED_DURATION_CONTINUOUS);


        while (opModeIsActive()) {
            double max;
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;
            double denominator = Math.max(Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw), 1);
            double leftFrontPower = (axial + lateral + yaw) / denominator;
            double rightFrontPower = (axial - lateral - yaw) / denominator;
            double leftBackPower = (axial - lateral + yaw) / denominator;
            double rightBackPower = (axial + lateral - yaw) / denominator;

            leftFrontDrive.setPower(leftFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightFrontDrive.setPower(rightFrontPower);
            rightBackDrive.setPower(rightBackPower);


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
            if (intakeOn == 1)
                intakeMotor.setPower(-1);
            else if (intakeOn == 2)
                intakeMotor.setPower(1);
            else
                intakeMotor.setPower(0);

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
                if (t <= 150) {
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

            if (gamepad1.leftBumperWasPressed()) {
                if (!shootingState) {
                    shootMotor.setVelocity(farLaunch);
                    shootingState = true;
                    gamepad1.setLedColor(0, 0, 1, LED_DURATION_CONTINUOUS);
                } else {
                    shootMotor.setVelocity(closeLaunch);
                    shootingState = false;
                    gamepad1.setLedColor(1, 0, 0, LED_DURATION_CONTINUOUS);
                }
            }
/*
            if (gamepad1.xWasPressed() && flyWheelOn){
                shootMotor.setVelocity(0);
            }
            else{
                if (!shootingState) {
                    shootMotor.setVelocity(farLaunch);
                    shootingState = true;
                    gamepad1.setLedColor(0, 0, 1, LED_DURATION_CONTINUOUS);
                } else {
                    shootMotor.setVelocity(closeLaunch);
                    shootingState = false;
                    gamepad1.setLedColor(1, 0, 0, LED_DURATION_CONTINUOUS);
                }

 */
            }
        }
    }









/*
            if (gamepad1.left_bumper){
                if (!shootingState){
                    shootMotor.setVelocity(farLaunch);
                     shootingState = true;
                    gamepad1.setLedColor(0,0,1, LED_DURATION_CONTINUOUS);
                }
                else{
                    shootMotor.setVelocity(closeLaunch);
                     shootingState = false;
                    gamepad1.setLedColor(1,0,0, LED_DURATION_CONTINUOUS);
                }
            }
            if(gamepad1.a && !aAlreadyPressed){
                motorOn = !motorOn;
                if(motorOn){
                    if (!shootingState){
                        shootMotor.setVelocity(farLaunch);
                        shootingState = true;
                        gamepad1.setLedColor(0,0,1, LED_DURATION_CONTINUOUS);
                    }
                    else{
                        shootMotor.setVelocity(closeLaunch);
                        shootingState = false;
                        gamepad1.setLedColor(1,0,0, LED_DURATION_CONTINUOUS);
                    }
                }
                else{
                    shootMotor.setVelocity(0);
                }
            }
            aAlreadyPressed = gamepad1.a;


            if(gamepad1.y && !yAlreadyPressed){
                motorOny = !motorOny;
                if(motorOny){
                    blockServo.setPosition(0.3);
                    pushServo.setPosition(0.3);
                }
                else{
                    blockServo.setPosition(0.8);
                    pushServo.setPosition(0.85);
                }
            }
            yAlreadyPressed = gamepad1.y;


 */

