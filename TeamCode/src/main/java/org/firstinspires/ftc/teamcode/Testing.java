package org.firstinspires.ftc.teamcode;



import static com.qualcomm.robotcore.hardware.Gamepad.LED_DURATION_CONTINUOUS;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.pedropathing.util.Timer;

@TeleOp(name = "Testing", group = "Linear Opmode")
public class Testing extends LinearOpMode {



    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    //Drive
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx intakeMotor = null;
    private DcMotorEx shootMotor = null;
    private Servo pushServo = null;
    private Servo blockServo = null;
    final double closeLaunch = 1000; //in ticks/second for the close goal.
    final double farLaunch = 1330;

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
        //Claw


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
        boolean intakeToggle = false;
        boolean aAlreadyPressed = false;
        boolean motorOn = false;
        boolean bAlreadyPressed = false;
        boolean motorOnb = false;
        boolean yAlreadyPressed = false;
        boolean motorOny = false;
        boolean xAlreadyPressed = false;
        boolean motorOnx = false;
        boolean shootingState = false;
        boolean shooterOn = true;



        pushServo.setPosition(0.9);
        blockServo.setPosition(0.9);

        waitForStart();
        runtime.reset();
        boolean isPushingManual = false;
        boolean isPushing = false;
        int shots = 0;
        ElapsedTime pushTimer = new ElapsedTime();
        ElapsedTime pushTimer1 = new ElapsedTime();
        shootMotor.setVelocity(closeLaunch);
        gamepad1.setLedColor(1,0,0, LED_DURATION_CONTINUOUS);



        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw), 1);

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.


            double leftFrontPower = (axial + lateral + yaw) / denominator;
            double rightFrontPower = (axial - lateral - yaw) / denominator;
            double leftBackPower = (axial - lateral +  yaw) / denominator;
            double rightBackPower = (axial + lateral - yaw) / denominator;






            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightFrontDrive.setPower(rightFrontPower);
            rightBackDrive.setPower(rightBackPower);


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

            if(gamepad1.left_trigger > 0.3 && !bAlreadyPressed){
                motorOnb = !motorOnb;
                if(motorOnb){
                    intakeMotor.setPower(-1);
                }
                else{
                    intakeMotor.setPower(0);
                }
            }
            bAlreadyPressed = gamepad1.left_trigger > 0.3;
/*
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

            if (gamepad1.right_trigger > 0.3 && !isPushing && shots == 0) {
                shots = 3;  // repeat 3 times
            }

// run the same sequence 3 times
            if (!isPushing && shots > 0) {
                isPushing = true;
                pushTimer1.reset();
                pushServo.setPosition(0.3);
                blockServo.setPosition(0.3);
            }

            if (isPushing) {
                if (pushTimer1.milliseconds() > 400) {
                    pushServo.setPosition(0.9);
                    blockServo.setPosition(0.9);
                    isPushing = false;
                    shots--;
                }
            }



            if (gamepad1.right_bumper && !isPushingManual) {
                // Start the sequence when Y is pressed
                isPushingManual = true;
                pushTimer.reset();
                pushServo.setPosition(0.3);
                blockServo.setPosition(0.3);// move to first position (up)
            }
            if (isPushingManual) {
                // After 300ms, return servo down

                if (pushTimer.milliseconds() > 400) {
                    pushServo.setPosition(0.9);
                    blockServo.setPosition(0.9);// move down
                    isPushingManual = false;           // end the sequence
                }
            }

            if(gamepad1.x && !xAlreadyPressed){
                motorOnx = !motorOnx;
                if(motorOnx){
                    intakeMotor.setPower(1);
                }
                else{
                    intakeMotor.setPower(0);
                }
            }
            xAlreadyPressed = gamepad1.x;










        }
    }
}
