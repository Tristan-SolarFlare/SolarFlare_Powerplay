/*
package org.firstinspires.ftc.teamcode.apriltagdetection;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

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

public class GlobalPreloadConeAuto extends LinearOpMode{
    private int target;

    Servo arm1 = hardwareMap.servo.get("arm");
    Servo arm2 = hardwareMap.servo.get("arm2");

    DcMotorEx slide1 = hardwareMap.get(DcMotorEx.class, "slide1");
    DcMotorEx slide2 = hardwareMap.get(DcMotorEx.class, "slide2");

    Servo claw = hardwareMap.servo.get("claw");

    Servo wrist = hardwareMap.servo.get("wrist");
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

    class SlidePID implements Action {
        public boolean run(@NonNull TelemetryPacket packet) {
            double Kp = 0.015;
            telemetry.addData("Slide1 Position", slide1.getCurrentPosition());
            telemetry.addData("Slide2 Position", slide1.getCurrentPosition());
            telemetry.update();
            // Power applied if error is above maxErrorThreshold
            double maxAdjustingPower = 1;
            // minimum error for max power correction, else it will
            int maxErrorThreshold = 67;
            double slide1power = 0.0;
            double slide2power = 0.0;
            int maxAcceptableError = 15;

            if (Math.abs(slide1.getCurrentPosition() - target) > maxAcceptableError) {
                // Calculates amount of ticks off slide1 is from target
                double error1 = -(target - slide1.getCurrentPosition());

                if ((Math.abs(error1) > maxErrorThreshold)) {
                    // signum finds the sign of the error
                    slide1power = Math.signum(error1) * maxAdjustingPower;
                } else {
                    // slidepower based off fraction of error times maxAdjusting power
                    slide1power = (error1 / maxErrorThreshold) * maxAdjustingPower;
                }
                slide1.setPower(slide1power);
            } else {
                slide1.setPower(0.0);
            }

            if (Math.abs(slide2.getCurrentPosition() - target) > maxAcceptableError) {
                // We need a new slide2 power that will correct for error
                // Calculates amount of ticks off slide2 is from target
                double error2 = target - slide2.getCurrentPosition();
                if ((Math.abs(error2) > maxErrorThreshold)) {
                    slide2power = Math.signum(error2) * maxAdjustingPower;
                } else {
                    slide2power = (error2 / maxErrorThreshold) * maxAdjustingPower;
                }
                slide2.setPower(slide2power);
            } else {
                slide2.setPower(0.0);
            }
            telemetry.addData("Slide1 Position", slide1.getCurrentPosition());
            telemetry.addData("Slide2 Position", slide1.getCurrentPosition());
            telemetry.addData("Slide1 Power", slide1power);
            telemetry.addData("Slide2 Power", slide2power);
            telemetry.update();
            return true;
        }
    }
    class DepositPosition implements Action{
        public boolean isOver;
        public boolean run(@NonNull TelemetryPacket packet){
            target=440;
            if(!isOver){
                return true;
            }
            else{
                return false;
            }
        }
    }
    class FlipArm implements Action{
        public boolean run(@NonNull TelemetryPacket packet){
            arm1.setPosition(0.96);
            arm2.setPosition(0.96);
            wrist.setPosition(0.3);
            sleep(300);
            return false;
        }
    }
}
class RetractionSequence implements Action{
    public boolean isOver;
    public boolean run(@NonNull TelemetryPacket packet){
        target=0;
        if(!isOver){
            return true;
        }
        else{
            return false;
        }
    }
}
    @Override
    public void runOpMode(){
        arm1.setDirection(Servo.Direction.REVERSE);
        slide1.setDirection(DcMotorSimple.Direction.REVERSE);
        slide2.setDirection(DcMotorSimple.Direction.FORWARD);

        // Ensures that when no power is set on the motors they will hold their position
        slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm1.setPosition(0.02);
        arm2.setPosition(0.02);
        wrist.setPosition(0.91);
        claw.setPosition(0.3);
    }
}

        }

 */
