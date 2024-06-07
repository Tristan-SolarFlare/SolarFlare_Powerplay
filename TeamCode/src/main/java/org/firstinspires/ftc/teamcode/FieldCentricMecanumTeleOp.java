package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class FieldCentricMecanumTeleOp extends LinearOpMode {
    // target position of linear slides in ticks (not related to actual position)

    @Override
    public void runOpMode() throws InterruptedException {

        final double MOTOR_PPR = 145.1; // aka ticks per rotation
        final double TICKS_PER_CM = (int) Math.round(145.1 / 12);

        // Maps motors and servos
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");


        Servo arm1 = hardwareMap.servo.get("arm");
        Servo arm2 = hardwareMap.servo.get("arm2");

        Servo claw = hardwareMap.servo.get("claw");

        Servo wrist = hardwareMap.servo.get("wrist");

        DcMotorEx slide1 = hardwareMap.get(DcMotorEx.class,"slide1");
        DcMotorEx slide2 = hardwareMap.get(DcMotorEx.class,"slide2");

        // Retrieves the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");

        // Orients motors to allow for forward movement
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Ensures that when no power is set on the motors they will hold their position
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Our robots orientation
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));

        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        // Initalize slides
        slide1.setDirection(DcMotorSimple.Direction.REVERSE);
        slide2.setDirection(DcMotorSimple.Direction.FORWARD);
        slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm1.setDirection(Servo.Direction.REVERSE);
        
        //Set slides Kp value
        double Kp = 0.015;
        // Initialize default  positions

        double linearSlidesTarget = 0;

        double arm1pos = 0.02;
        double arm2pos = 0.02;

        double wristpos= 0.91;

        double slidePower = 0.2;



        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {


            double y = -gamepad1.left_stick_y; // y stick value should be reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }


            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;


            double liftSpeed = 250; // ticks per second

            if (gamepad1.left_trigger>0.5) { // Checks for left bumper input, slows all motors by 50%
                frontLeftPower = 0.5 * (rotY + rotX + rx) / denominator;
                backLeftPower = 0.5 * (rotY - rotX + rx) / denominator;
                frontRightPower = 0.5 * (rotY - rotX - rx) / denominator;
                backRightPower = 0.5 * (rotY + rotX - rx) / denominator;
            }
            if (gamepad1.a){ //reset to pickup position
                linearSlidesTarget=0;
                wristpos=0.91;
                arm1pos=0.02;
                arm2pos=0.02;
            }else if (gamepad1.b){ //low junction
                linearSlidesTarget=440; //rough estimate - we have to tune the encoder positions
            }else if(gamepad1.x){ //mid junction
                linearSlidesTarget = 250;// rough estimate - this has to be changed, the reason this is lower than low junction is because we get the added distance from the flip
                arm1pos = 0.96;
                arm2pos=0.96;
                wristpos=0.2;
            }else if(gamepad1.y){ //high junction
                linearSlidesTarget = 430; //tune encoders
                //add code to flip servos
                arm1pos = 0.96;
                arm2pos=0.96;
                wristpos=0.2;
            }
            if(gamepad1.left_bumper){
                //slide1.setPower(0.75);
                //slide2.setPower(0.75);

                if (linearSlidesTarget<645){
                    linearSlidesTarget= linearSlidesTarget + 15;
                }

            }else if(gamepad1.right_bumper){
                //slide1.setPower(-0.5);
                //slide2.setPower(-0.5);
                if (linearSlidesTarget > -1){
                    linearSlidesTarget = linearSlidesTarget - 15;
                }
            }
            claw.setPosition(gamepad1.right_trigger * 0.3);
            arm1.setPosition(arm1pos);
            arm2.setPosition(arm2pos);
            wrist.setPosition((wristpos));

            double error1=-(linearSlidesTarget-slide1.getCurrentPosition());
            double slide1power;
            if ((Math.abs(error1)>80)){
                slide1power = (0.75*error1);
            }
            else {
                slide1power = (Kp*error1);
            }
            slide1.setPower(slide1power);

            double error2=linearSlidesTarget-slide2.getCurrentPosition();
            double slide2power;
            if ((Math.abs(error2)>80)){
                slide2power = 0.75*error2;
            }

            else {
                slide2power = Kp*error2;
            }
            slide2.setPower(slide2power);

            telemetry.addData("Slide1:",slide1.getCurrentPosition());
            telemetry.addData("Slide2:",slide2.getCurrentPosition());
            telemetry.addData("Target:",linearSlidesTarget);
            telemetry.addData("Error1:",error1);
            telemetry.addData("Error2:",error2);
            telemetry.addData("Slide 1 Power:",slide1power);
            telemetry.addData("slide 2 power:",slide2power);
            telemetry.addData("Wrist position:",wrist.getPosition());
            telemetry.update();
            // set powers for driving
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }
    }
}