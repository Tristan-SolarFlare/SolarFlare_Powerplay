/* https://drive.google.com/file/d/1GAEcuIJ_Pu-IoopSYptUfXAdABRY6eKw/view
    upload above file to control hub. Drag into "FIRST" folder.
 */

package org.firstinspires.ftc.teamcode.apriltagdetection;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;
import androidx.appcompat.widget.VectorEnabledTintResources;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

import kotlin.math.UMathKt;

@Autonomous

public class MidCycleAuto extends LinearOpMode
{

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!

    // The x-axis focal length of the camera in pixels
    double fx = 578.272;
    // The y-axis focal length of the camera in pixels
    double fy = 578.272;
    // The x-axis optical center of the camera in pixels
    double cx = 402.145;
    // The y-axis optical center of the camera in pixels.
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // April tag IDs
    int leftTag = 17;
    int middleTag = 18;
    int rightTag = 19;

    AprilTagDetection tagOfInterest = null;

    public class Lift {
        // mid=120
        // topcone=150
        // second=120
        // third=85
        // fourth=40
        // last=0

        //NOTE: UPDATE TIMING ACCORDINGLY LATER
        public class GrabCone1 implements Action {
            double target;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                DcMotorEx slide1 = hardwareMap.get(DcMotorEx.class, "slide1");
                DcMotorEx slide2 = hardwareMap.get(DcMotorEx.class, "slide2");

                slide1.setDirection(DcMotorSimple.Direction.REVERSE);
                slide2.setDirection(DcMotorSimple.Direction.FORWARD);

                // Ensures that when no power is set on the motors they will hold their position
                slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                Servo arm1 = hardwareMap.servo.get("arm");
                Servo arm2 = hardwareMap.servo.get("arm2");

                arm1.setDirection(Servo.Direction.REVERSE);


                Servo claw = hardwareMap.servo.get("claw");

                Servo wrist = hardwareMap.servo.get("wrist");
                telemetry.addData("Slide1 Position", slide1.getCurrentPosition());
                telemetry.addData("Slide2 Position", slide1.getCurrentPosition());
                telemetry.update();

                double Kp = 0.015;
                arm1.setPosition(0.02);
                arm2.setPosition(0.02);
                wrist.setPosition(0.91);
                claw.setPosition(0);
                sleep(2000);
                target = 170;

                double slide1power;
                // Calculates amount of ticks off slide1 is from target
                double error1 = -(target - slide1.getCurrentPosition()); // Error is negative because slide1 needs to reverse direction

                // If error1 is greater than 80, correct by faster speed, else correct by usual speed
                if ((Math.abs(error1) > 80)) {
                    slide1power = (0.75 * error1);
                } else {
                    slide1power = (Kp * error1);
                }
                slide1.setPower(slide1power);

                // We need a new slide2 power that will correct for error
                double slide2power;

                // Calculates amount of ticks off slide2 is from target
                double error2 = target - slide2.getCurrentPosition();

                // If error1 is greater than 80, correct by faster speed, else correct by usual speed
                if ((Math.abs(error2) > 80)) {
                    slide2power = 0.75 * error2;
                } else {
                    slide2power = Kp * error2;
                }
                slide2.setPower(slide2power);
                telemetry.addData("Slide1 Position", slide1.getCurrentPosition());
                telemetry.addData("Slide2 Position", slide1.getCurrentPosition());
                telemetry.update();
                if (slide1.getCurrentPosition() > 140) {
                    return false;
                }
                return true;
            }
        }
        public class GrabCone2 implements Action {
            double target;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                DcMotorEx slide1 = hardwareMap.get(DcMotorEx.class, "slide1");
                DcMotorEx slide2 = hardwareMap.get(DcMotorEx.class, "slide2");

                slide1.setDirection(DcMotorSimple.Direction.REVERSE);
                slide2.setDirection(DcMotorSimple.Direction.FORWARD);

                // Ensures that when no power is set on the motors they will hold their position
                slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                Servo arm1 = hardwareMap.servo.get("arm");
                Servo arm2 = hardwareMap.servo.get("arm2");

                arm1.setDirection(Servo.Direction.REVERSE);


                Servo claw = hardwareMap.servo.get("claw");

                Servo wrist = hardwareMap.servo.get("wrist");
                telemetry.addData("Slide1 Position", slide1.getCurrentPosition());
                telemetry.addData("Slide2 Position", slide1.getCurrentPosition());
                telemetry.update();

                double Kp = 0.015;
                arm1.setPosition(0.02);
                arm2.setPosition(0.02);
                wrist.setPosition(0.91);
                claw.setPosition(0);

                return true;
            }
        }
        public class GrabCone3 implements Action {
            double target;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                DcMotorEx slide1 = hardwareMap.get(DcMotorEx.class, "slide1");
                DcMotorEx slide2 = hardwareMap.get(DcMotorEx.class, "slide2");

                slide1.setDirection(DcMotorSimple.Direction.REVERSE);
                slide2.setDirection(DcMotorSimple.Direction.FORWARD);

                // Ensures that when no power is set on the motors they will hold their position
                slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                Servo arm1 = hardwareMap.servo.get("arm");
                Servo arm2 = hardwareMap.servo.get("arm2");

                arm1.setDirection(Servo.Direction.REVERSE);


                Servo claw = hardwareMap.servo.get("claw");

                Servo wrist = hardwareMap.servo.get("wrist");
                telemetry.addData("Slide1 Position", slide1.getCurrentPosition());
                telemetry.addData("Slide2 Position", slide1.getCurrentPosition());
                telemetry.update();

                double Kp = 0.03; //tune this
                arm1.setPosition(0.02);
                arm2.setPosition(0.02);
                wrist.setPosition(0.91);
                claw.setPosition(0);
                sleep(2000);
                target = 115;

                double slide1power;
                // Calculates amount of ticks off slide1 is from target
                double error1 = -(target - slide1.getCurrentPosition()); // Error is negative because slide1 needs to reverse direction

                // If error1 is greater than 80, correct by faster speed, else correct by usual speed
                if ((Math.abs(error1) > 80)) {
                    slide1power = (0.75 * error1);
                } else {
                    slide1power = (Kp * error1);
                }
                slide1.setPower(slide1power);

                // We need a new slide2 power that will correct for error
                double slide2power;

                // Calculates amount of ticks off slide2 is from target
                double error2 = target - slide2.getCurrentPosition();

                // If error1 is greater than 80, correct by faster speed, else correct by usual speed
                if ((Math.abs(error2) > 80)) {
                    slide2power = 0.75 * error2;
                } else {
                    slide2power = Kp * error2;
                }
                slide2.setPower(slide2power);
                telemetry.addData("Slide1 Position", slide1.getCurrentPosition());
                telemetry.addData("Slide2 Position", slide1.getCurrentPosition());
                telemetry.update();
                if (slide1.getCurrentPosition() < 80) {
                    return false;
                }
                return true;
            }
        }
        public class GrabCone4 implements Action {
            double target;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                DcMotorEx slide1 = hardwareMap.get(DcMotorEx.class, "slide1");
                DcMotorEx slide2 = hardwareMap.get(DcMotorEx.class, "slide2");

                slide1.setDirection(DcMotorSimple.Direction.REVERSE);
                slide2.setDirection(DcMotorSimple.Direction.FORWARD);

                // Ensures that when no power is set on the motors they will hold their position
                slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                Servo arm1 = hardwareMap.servo.get("arm");
                Servo arm2 = hardwareMap.servo.get("arm2");

                arm1.setDirection(Servo.Direction.REVERSE);


                Servo claw = hardwareMap.servo.get("claw");

                Servo wrist = hardwareMap.servo.get("wrist");
                telemetry.addData("Slide1 Position", slide1.getCurrentPosition());
                telemetry.addData("Slide2 Position", slide1.getCurrentPosition());
                telemetry.update();

                double Kp = 0.03; //tune this
                arm1.setPosition(0.02);
                arm2.setPosition(0.02);
                wrist.setPosition(0.91);
                claw.setPosition(0);
                sleep(2000);
                target = 70;

                double slide1power;
                // Calculates amount of ticks off slide1 is from target
                double error1 = -(target - slide1.getCurrentPosition()); // Error is negative because slide1 needs to reverse direction

                // If error1 is greater than 80, correct by faster speed, else correct by usual speed
                if ((Math.abs(error1) > 80)) {
                    slide1power = (0.75 * error1);
                } else {
                    slide1power = (Kp * error1);
                }
                slide1.setPower(slide1power);

                // We need a new slide2 power that will correct for error
                double slide2power;

                // Calculates amount of ticks off slide2 is from target
                double error2 = target - slide2.getCurrentPosition();

                // If error1 is greater than 80, correct by faster speed, else correct by usual speed
                if ((Math.abs(error2) > 80)) {
                    slide2power = 0.75 * error2;
                } else {
                    slide2power = Kp * error2;
                }
                slide2.setPower(slide2power);
                telemetry.addData("Slide1 Position", slide1.getCurrentPosition());
                telemetry.addData("Slide2 Position", slide1.getCurrentPosition());
                telemetry.update();
                if (slide1.getCurrentPosition() < 45) {
                    return false;
                }
                return true;
            }
        }
        public class GrabCone5 implements Action {
            double target;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                DcMotorEx slide1 = hardwareMap.get(DcMotorEx.class, "slide1");
                DcMotorEx slide2 = hardwareMap.get(DcMotorEx.class, "slide2");

                slide1.setDirection(DcMotorSimple.Direction.REVERSE);
                slide2.setDirection(DcMotorSimple.Direction.FORWARD);

                // Ensures that when no power is set on the motors they will hold their position
                slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                Servo arm1 = hardwareMap.servo.get("arm");
                Servo arm2 = hardwareMap.servo.get("arm2");

                arm1.setDirection(Servo.Direction.REVERSE);


                Servo claw = hardwareMap.servo.get("claw");

                Servo wrist = hardwareMap.servo.get("wrist");
                telemetry.addData("Slide1 Position", slide1.getCurrentPosition());
                telemetry.addData("Slide2 Position", slide1.getCurrentPosition());
                telemetry.update();

                double Kp = 0.03; //tune this
                arm1.setPosition(0.02);
                arm2.setPosition(0.02);
                wrist.setPosition(0.91);
                claw.setPosition(0);
                sleep(2000);
                target = 0;

                double slide1power;
                // Calculates amount of ticks off slide1 is from target
                double error1 = -(target - slide1.getCurrentPosition()); // Error is negative because slide1 needs to reverse direction

                // If error1 is greater than 80, correct by faster speed, else correct by usual speed
                if ((Math.abs(error1) > 80)) {
                    slide1power = (0.75 * error1);
                } else {
                    slide1power = (Kp * error1);
                }
                slide1.setPower(slide1power);

                // We need a new slide2 power that will correct for error
                double slide2power;

                // Calculates amount of ticks off slide2 is from target
                double error2 = target - slide2.getCurrentPosition();

                // If error1 is greater than 80, correct by faster speed, else correct by usual speed
                if ((Math.abs(error2) > 80)) {
                    slide2power = 0.75 * error2;
                } else {
                    slide2power = Kp * error2;
                }
                slide2.setPower(slide2power);
                telemetry.addData("Slide1 Position", slide1.getCurrentPosition());
                telemetry.addData("Slide2 Position", slide1.getCurrentPosition());
                telemetry.update();
                if (slide1.getCurrentPosition() > 5) {
                    return false;
                }
                return true;
            }
        }


        public class PickUpCone implements Action {
            double target;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                DcMotorEx slide1 = hardwareMap.get(DcMotorEx.class, "slide1");
                DcMotorEx slide2 = hardwareMap.get(DcMotorEx.class, "slide2");

                slide1.setDirection(DcMotorSimple.Direction.REVERSE);
                slide2.setDirection(DcMotorSimple.Direction.FORWARD);

                // Ensures that when no power is set on the motors they will hold their position
                slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                Servo arm1 = hardwareMap.servo.get("arm");
                Servo arm2 = hardwareMap.servo.get("arm2");

                arm1.setDirection(Servo.Direction.REVERSE);


                Servo claw = hardwareMap.servo.get("claw");

                Servo wrist = hardwareMap.servo.get("wrist");
                telemetry.addData("Slide1 Position", slide1.getCurrentPosition());
                telemetry.addData("Slide2 Position", slide1.getCurrentPosition());
                telemetry.update();

                double Kp = 0.015;
                arm1.setPosition(0.02);
                arm2.setPosition(0.02);
                wrist.setPosition(0.91);
                claw.setPosition(0.3);
                sleep(2000); //tune this
                target = 350;

                double slide1power;
                // Calculates amount of ticks off slide1 is from target
                double error1 = -(target - slide1.getCurrentPosition()); // Error is negative because slide1 needs to reverse direction

                // If error1 is greater than 80, correct by faster speed, else correct by usual speed
                if ((Math.abs(error1) > 80)) {
                    slide1power = (0.75 * error1);
                } else {
                    slide1power = (Kp * error1);
                }
                slide1.setPower(slide1power);

                // We need a new slide2 power that will correct for error
                double slide2power;

                // Calculates amount of ticks off slide2 is from target
                double error2 = target - slide2.getCurrentPosition();

                // If error1 is greater than 80, correct by faster speed, else correct by usual speed
                if ((Math.abs(error2) > 80)) {
                    slide2power = 0.75 * error2;
                } else {
                    slide2power = Kp * error2;
                }
                slide2.setPower(slide2power);
                telemetry.addData("Slide1 Position", slide1.getCurrentPosition());
                telemetry.addData("Slide2 Position", slide1.getCurrentPosition());
                telemetry.update();
                if (slide1.getCurrentPosition() > 320) {
                    return false;
                }
                return false;
            }
        }


        public class DepositPosition implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                DcMotorEx slide1 = hardwareMap.get(DcMotorEx.class, "slide1");
                DcMotorEx slide2 = hardwareMap.get(DcMotorEx.class, "slide2");

                slide1.setDirection(DcMotorSimple.Direction.REVERSE);
                slide2.setDirection(DcMotorSimple.Direction.FORWARD);

                // Ensures that when no power is set on the motors they will hold their position
                slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                Servo arm1 = hardwareMap.servo.get("arm");
                Servo arm2 = hardwareMap.servo.get("arm2");

                arm1.setDirection(Servo.Direction.REVERSE);


                Servo claw = hardwareMap.servo.get("claw");

                Servo wrist = hardwareMap.servo.get("wrist");
                telemetry.addData("Slide1 Position", slide1.getCurrentPosition());
                telemetry.addData("Slide2 Position", slide1.getCurrentPosition());
                telemetry.update();

                double Kp = 0.015;
                double target;
                arm1.setPosition(0.02);
                arm2.setPosition(0.02);
                wrist.setPosition(0.91);
                claw.setPosition(0.3);
                sleep(300);
                arm1.setPosition(0.96);
                arm2.setPosition(0.96);
                wrist.setPosition(0.2);
                target = 140;
                double slide1power;
                // Calculates amount of ticks off slide1 is from target
                double error1 = -(target - slide1.getCurrentPosition()); // Error is negative because slide1 needs to reverse direction

                // If error1 is greater than 80, correct by faster speed, else correct by usual speed
                if ((Math.abs(error1) > 80)) {
                    slide1power = (0.75 * error1);
                } else {
                    slide1power = (Kp * error1);
                }
                slide1.setPower(slide1power);

                // We need a new slide2 power that will correct for error
                double slide2power;

                // Calculates amount of ticks off slide2 is from target
                double error2 = target - slide2.getCurrentPosition();

                // If error1 is greater than 80, correct by faster speed, else correct by usual speed
                if ((Math.abs(error2) > 80)) {
                    slide2power = 0.75 * error2;
                } else {
                    slide2power = Kp * error2;
                }
                slide2.setPower(slide2power);
                telemetry.addData("Slide1 Position", slide1.getCurrentPosition());
                telemetry.addData("Slide2 Position", slide1.getCurrentPosition());
                telemetry.update();
                if (Math.abs(slide1.getCurrentPosition()-target)<20){
                    return false;

                }
                return true;
            }
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                DcMotorEx slide1 = hardwareMap.get(DcMotorEx.class, "slide1");
                DcMotorEx slide2 = hardwareMap.get(DcMotorEx.class, "slide2");

                slide1.setDirection(DcMotorSimple.Direction.REVERSE);
                slide2.setDirection(DcMotorSimple.Direction.FORWARD);

                // Ensures that when no power is set on the motors they will hold their position
                slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                Servo arm1 = hardwareMap.servo.get("arm");
                Servo arm2 = hardwareMap.servo.get("arm2");

                arm1.setDirection(Servo.Direction.REVERSE);


                Servo claw = hardwareMap.servo.get("claw");

                Servo wrist = hardwareMap.servo.get("wrist");
                telemetry.addData("Slide1 Position", slide1.getCurrentPosition());
                telemetry.addData("Slide2 Position", slide1.getCurrentPosition());
                telemetry.update();

                claw.setPosition(0);

                sleep(1000 );


                return false;
            }
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                DcMotorEx slide1 = hardwareMap.get(DcMotorEx.class, "slide1");
                DcMotorEx slide2 = hardwareMap.get(DcMotorEx.class, "slide2");

                slide1.setDirection(DcMotorSimple.Direction.REVERSE);
                slide2.setDirection(DcMotorSimple.Direction.FORWARD);

                // Ensures that when no power is set on the motors they will hold their position
                slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                Servo arm1 = hardwareMap.servo.get("arm");
                Servo arm2 = hardwareMap.servo.get("arm2");

                arm1.setDirection(Servo.Direction.REVERSE);


                Servo claw = hardwareMap.servo.get("claw");

                Servo wrist = hardwareMap.servo.get("wrist");
                telemetry.addData("Slide1 Position", slide1.getCurrentPosition());
                telemetry.addData("Slide2 Position", slide1.getCurrentPosition());
                telemetry.update();

                claw.setPosition(0.3);


                return false;
            }
        }

        public class RetractionSequence implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                DcMotorEx slide1 = hardwareMap.get(DcMotorEx.class, "slide1");
                DcMotorEx slide2 = hardwareMap.get(DcMotorEx.class, "slide2");

                slide1.setDirection(DcMotorSimple.Direction.REVERSE);
                slide2.setDirection(DcMotorSimple.Direction.FORWARD);

                // Ensures that when no power is set on the motors they will hold their position
                slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                Servo arm1 = hardwareMap.servo.get("arm");
                Servo arm2 = hardwareMap.servo.get("arm2");

                arm1.setDirection(Servo.Direction.REVERSE);


                Servo claw = hardwareMap.servo.get("claw");

                Servo wrist = hardwareMap.servo.get("wrist");
                telemetry.addData("Slide1 Position", slide1.getCurrentPosition());
                telemetry.addData("Slide2 Position", slide1.getCurrentPosition());
                telemetry.update();

                double Kp = 0.015;
                double target;
                sleep(1500);
                arm1.setPosition(0.02);
                arm2.setPosition(0.02);
                wrist.setPosition(0.91);
                sleep(600);
                claw.setPosition(0);
                target = 0;
                while (Math.abs(slide1.getCurrentPosition() - target) > 15 || Math.abs(slide2.getCurrentPosition() - target) > 15) {
                    double slide1power;
                    // Calculates amount of ticks off slide1 is from target
                    double error1 = -(target - slide1.getCurrentPosition()); // Error is negative because slide1 needs to reverse direction

                    // If error1 is greater than 80, correct by faster speed, else correct by usual speed
                    if ((Math.abs(error1) > 80)) {
                        slide1power = (0.75 * error1);
                    } else {
                        slide1power = (Kp * error1);
                    }
                    slide1.setPower(slide1power);

                    // We need a new slide2 power that will correct for error
                    double slide2power;

                    // Calculates amount of ticks off slide2 is from target
                    double error2 = target - slide2.getCurrentPosition();

                    // If error1 is greater than 80, correct by faster speed, else correct by usual speed
                    if ((Math.abs(error2) > 80)) {
                        slide2power = 0.75 * error2;
                    } else {
                        slide2power = Kp * error2;
                    }
                    slide2.setPower(slide2power);

                    telemetry.addData("Slide1 Position", slide1.getCurrentPosition());
                    telemetry.addData("Slide2 Position", slide1.getCurrentPosition());
                    telemetry.update();

                }
                sleep(1000);
                return false;
            }
        }


        public Action GrabCone1() {
            return new GrabCone1();
        }
        public Action GrabCone2() {
            return new GrabCone2();
        }
        public Action GrabCone3() {
            return new GrabCone3();
        }
        public Action GrabCone4() {
            return new GrabCone4();
        }
        public Action GrabCone5() {
            return new GrabCone5();
        }
        public Action PickUpCone() {
            return new PickUpCone();
        }

        public Action DepositPosition() {
            return new DepositPosition();
        }

        public Action CloseClaw() {
            return new CloseClaw();

        }

        public Action OpenClaw(){
            return new OpenClaw();
        }

        public Action RetractionSequence(){
            return new RetractionSequence();
        }
    }

    @Override
    public void runOpMode()    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);

        DcMotorEx slide1 = hardwareMap.get(DcMotorEx.class,"slide1");
        DcMotorEx slide2 = hardwareMap.get(DcMotorEx.class,"slide2");

        slide1.setDirection(DcMotorSimple.Direction.REVERSE);
        slide2.setDirection(DcMotorSimple.Direction.FORWARD);

        // Ensures that when no power is set on the motors they will hold their position
        slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */

        Action DriveInitialDeposit;
        Action DriveToIntakeFromInitialDeposit;
        Action DriveToIntake;
        Action DriveToDeposit;
        Action ParkZone1;
        Action ParkZone2;
        Action ParkZone3;


        //set staring position, unit is inches
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(11, 36, Math.toRadians(0)));

        DriveInitialDeposit = drive.actionBuilder(drive.pose)
                .turn(Math.toRadians(90))
                .strafeTo(new Vector2d(45,36))
                .waitSeconds(0.4)
                .build();

        DriveToIntakeFromInitialDeposit = drive.actionBuilder(new Pose2d(45,36,Math.toRadians(90)))
                .strafeTo(new Vector2d(57,36))
                .strafeTo(new Vector2d(56.5,12))
                .waitSeconds(0.2)
                .build();

        DriveToIntake = drive.actionBuilder(new Pose2d(55,38,Math.toRadians(120)))
                .strafeToLinearHeading(new Vector2d(56,12),Math.toRadians(90))
                .waitSeconds(0.2)
                .build();

        DriveToDeposit = drive.actionBuilder(new Pose2d(56,12,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(55,38),Math.toRadians(120))
                .waitSeconds(0.4)
                .build();

        ParkZone1 = drive.actionBuilder(new Pose2d(55,38,Math.toRadians(120)))
                .turn(Math.toRadians(-120))
                .strafeTo(new Vector2d(36,36))
                .strafeTo(new Vector2d(36,60))
                .build();

        ParkZone2 = drive.actionBuilder(new Pose2d(55,38,Math.toRadians(120)))
                .turn(Math.toRadians(-120))
                .build();

        ParkZone3 = drive.actionBuilder(new Pose2d(55,38,Math.toRadians(120)))
                .turn(Math.toRadians(-120))
                .strafeTo(new Vector2d(36,36))
                .strafeTo(new Vector2d(36,12))
                .build();

        boolean tagFound = false;

        waitForStart();

        if (isStopRequested()) return;

        while (!tagFound)
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == leftTag || tag.id == middleTag || tag.id == rightTag)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        // Performs actions based on tag detection
        // If the camera cannot find a tag, it will default to performing the left tag operation


        Action trajectoryActionChosen = null  ;

        if(tagOfInterest == null){

            // Runs autonomous and parks in zone 2
            // Runs if camera does not detect april tag
            trajectoryActionChosen = ParkZone2;
        }
        if(tagOfInterest.id == leftTag){

            // Runs autonomous and parks in zone 1
            trajectoryActionChosen = ParkZone1;
        }
        else if (tagOfInterest.id == middleTag){

            // Runs autonomous and parks in zone 2
            trajectoryActionChosen = ParkZone2;
        }
        else if (tagOfInterest.id == rightTag){

            // Runs autonomous and parks in zone 3
            trajectoryActionChosen = ParkZone3;
        }
        Lift lift= new Lift();
        Actions.runBlocking(
                new SequentialAction(

                        lift.CloseClaw(),

                        new ParallelAction(
                                DriveInitialDeposit,
                                lift.DepositPosition()
                        ),

                        lift.OpenClaw(),

                        new ParallelAction(
                                DriveToIntakeFromInitialDeposit,
                                lift.GrabCone1()
                        ),

                        lift.CloseClaw(),
                        lift.PickUpCone(),

                        new ParallelAction(
                                DriveToDeposit,
                                lift.DepositPosition()
                        ),

                        lift.OpenClaw(),

                        new ParallelAction(
                                DriveToIntake,
                                lift.GrabCone2()
                        ),

                        lift.CloseClaw(),
                        lift.PickUpCone(),

                        new ParallelAction(
                                DriveToDeposit,
                                lift.DepositPosition()
                        ),

                        lift.OpenClaw(),

                        new ParallelAction(
                                DriveToIntake,
                                lift.GrabCone3()
                        ),

                        lift.CloseClaw(),
                        lift.PickUpCone(),

                        new ParallelAction(
                                DriveToDeposit,
                                lift.DepositPosition()
                        ),

                        lift.OpenClaw(),

                        new ParallelAction(
                                DriveToIntake,
                                lift.GrabCone4()
                        ),

                        lift.CloseClaw(),
                        lift.PickUpCone(),

                        new ParallelAction(
                                DriveToDeposit,
                                lift.DepositPosition()
                        ),

                        lift.OpenClaw(),

                        new ParallelAction(
                                DriveToIntake,
                                lift.GrabCone5()
                        ),

                        lift.CloseClaw(),
                        lift.PickUpCone(),

                        new ParallelAction(
                                DriveToDeposit,
                                lift.DepositPosition()
                        ),
                        lift.OpenClaw(),


                        new ParallelAction(
                                trajectoryActionChosen,
                                lift.RetractionSequence()
                        )



                        //I JUST REALIZED THIS IS WRONG, WILL BE ADDING DIFFERENT SEQUENCES LATER

                )
        );


        while (opModeIsActive()) {sleep(20);}
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle));
    }
}