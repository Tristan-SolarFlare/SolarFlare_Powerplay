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
    double target = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        double Kp = 0.015;


        double arm1pos = 0.30;
        double arm2pos = 0.30;

        double slidePower = 0.2;

        final double motorPPR = 145.1;
        final double ticksPerCM = (int) Math.round(145.1 / 12);

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

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        //initalize slides
        slide1.setDirection(DcMotorSimple.Direction.REVERSE);
        slide2.setDirection(DcMotorSimple.Direction.FORWARD);
        slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm1.setDirection(Servo.Direction.REVERSE);

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
                target=0;
            }else if (gamepad1.b){ //low junction
                target=320; //rough estimate - we have to tune the encoder positions
            }else if(gamepad1.x){ //mid junction
                target = 150;// rough estimate - this has to be changed, the reason this is lower than low junction is because we get the added distance from the flip
                arm1pos = 0.7;
            }else if(gamepad1.y){ //high junction
                target = 440; //tune encoders
                //add code to flip servos
            }
            if(gamepad1.left_bumper){
                //slide1.setPower(0.75);
                //slide2.setPower(0.75);

                if (target<645){
                    target= target + 15;
                }

            }else if(gamepad1.right_bumper){
                //slide1.setPower(-0.5);
                //slide2.setPower(-0.5);
                if (target > -1){
                    target = target - 15;
                }
            }
            claw.setPosition(gamepad1.right_trigger * 0.3);

            //arm2pos=arm1pos-0; //tune this
            arm1.setPosition(arm1pos);
            arm2.setPosition(arm2pos);

            double error1=-(target-slide1.getCurrentPosition());
            double slide1power;
            if ((Math.abs(error1)>80)){
                slide1power = (0.75*error1);
            }
            else {
                slide1power = (Kp*error1);
            }
            slide1.setPower(slide1power);

            double error2=target-slide2.getCurrentPosition();
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
            telemetry.addData("Target:",target);
            telemetry.addData("Error1:",error1);
            telemetry.addData("Error2:",error2);
            telemetry.addData("Slide 1 Power:",slide1power);
            telemetry.addData("slide 2 power:",slide2power);

            telemetry.update();
            // set powers for driving
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }
    }
}