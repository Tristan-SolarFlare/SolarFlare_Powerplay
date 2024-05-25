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


@TeleOp
public class FieldCentricMecanumTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        double Kp = 0.02;

        double slidePower = 0.2;

        final double motorPPR = 145.1;
        final double ticksPerCM = (int) Math.round(145.1 / 12);


        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        DcMotorEx slide1 = hardwareMap.get(DcMotorEx.class,"slide1");
        DcMotorEx slide2 = hardwareMap.get(DcMotorEx.class,"slide2");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        //initalize slides
        slide1.setDirection(DcMotorSimple.Direction.FORWARD);
        slide2.setDirection(DcMotorSimple.Direction.FORWARD);
        slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        double target = 0;

        if (isStopRequested()) return;

        while (opModeIsActive()) {


            double y = -gamepad1.left_stick_y; // y stick value should be reversed
            double x = gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

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
            double lastTime = System.currentTimeMillis()/1000d;

            if (gamepad1.left_bumper) { // Checks for left bumper input, slows all motors by 50%
                frontLeftPower = 0.5 * (rotY + rotX + rx) / denominator;
                backLeftPower = 0.5 * (rotY - rotX + rx) / denominator;
                frontRightPower = 0.5 * (rotY - rotX - rx) / denominator;
                backRightPower = 0.5 * (rotY + rotX - rx) / denominator;
            }else if (gamepad1.a){ //reset to pickup position
                target=0;
                //commit: add code to set servos back to norm position.
            }else if (gamepad1.b){ //low junction
                target=100; //rough estimate - we have to tune the encoder positions
            }else if(gamepad1.x){ //mid junction
                target = 75;// rough estimate - this has to be changed, the reason this is lower than low junction is because we get the added distance from the flip
                //commit: add code to flip servos
            }else if(gamepad1.y){ //high junction
                target = 150; //tune encoders
                //add code to flip servos
            }
            /*
            double currentTime = System.currentTimeMillis()/1000d;
            //double deltaTime = currentTime - lastTime;
            // lastTime = currentTime;
            // adjust target based on game pad inputs
            double slideinput = (gamepad1.left_trigger-gamepad1.right_trigger);

            target += (slideinput * currentTime  * liftSpeed) % 145.1;

            slide1.setPower(0.2*slideinput);
            slide2.setPower(0.2*slideinput);


            double error1 = target - slide1.getCurrentPosition();
            double error2 = target - slide2.getCurrentPosition();

            slide1.setPower(error1*Kp);
            slide2.setPower(error2*Kp);

             */
            double slideinput = (gamepad1.left_trigger-gamepad1.right_trigger);

            slide1.setPower(0.75*slideinput);
            slide2.setPower(0.75*slideinput);

            telemetry.addData("Slide1:",slide1.getCurrentPosition());
            telemetry.addData("Slide2:",slide2.getCurrentPosition());

            // set powers for driving
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }
    }
}