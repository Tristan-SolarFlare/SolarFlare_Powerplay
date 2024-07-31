package org.firstinspires.ftc.teamcode.apriltagdetection;

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

@Autonomous

public class PreloadConeAuto extends LinearOpMode
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
        DcMotorEx slide1 = hardwareMap.get(DcMotorEx.class, "slide1");
        DcMotorEx slide2 = hardwareMap.get(DcMotorEx.class, "slide2");
        public Lift(){
            slide1.setDirection(DcMotorSimple.Direction.REVERSE);
            slide2.setDirection(DcMotorSimple.Direction.FORWARD);

            // Ensures that when no power is set on the motors they will hold their position
            slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        public class DepositPosition implements Action {
            private int target;

            public DepositPosition(int target) {
                this.target = target;
            }
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

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
                wrist.setPosition(0.3);
                target = 440;
                // Power applied if error is above maxErrorThreshold
                double maxAdjustingPower = 1;
                // minimum error for max power correction, else it will
                int maxErrorThreshold  = 67;
                double slide1power = 0.0;
                double slide2power = 0.0;
                int maxAcceptableError = 15;
                while (Math.abs(slide1.getCurrentPosition() - target) > maxAcceptableError ||
                        Math.abs(slide2.getCurrentPosition() - target) > maxAcceptableError) {

                    if(Math.abs(slide1.getCurrentPosition() - target) > maxAcceptableError) {
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

                    if(Math.abs(slide2.getCurrentPosition() - target) > maxAcceptableError) {
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
                }
                slide1.setPower(-0.05);
                slide2.setPower(0.05);
                sleep(1000);
                return false;
            }
        }

        public class RetractionSequence implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

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
                while (Math.abs(slide1.getCurrentPosition() - target) > 5 || Math.abs(slide2.getCurrentPosition() - target) > 5) {
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


        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                Servo arm1 = hardwareMap.servo.get("arm");
                Servo arm2 = hardwareMap.servo.get("arm2");

                arm1.setDirection(Servo.Direction.REVERSE);


                Servo claw = hardwareMap.servo.get("claw");

                Servo wrist = hardwareMap.servo.get("wrist");
                telemetry.addData("Slide1 Position", slide1.getCurrentPosition());
                telemetry.addData("Slide2 Position", slide1.getCurrentPosition());
                telemetry.update();

                claw.setPosition(0);

                sleep(1000);


                return false;
            }
        }

        public class CloseClaw implements Action {
            @Override

            public boolean run(@NonNull TelemetryPacket packet) {

                Servo arm1 = hardwareMap.servo.get("arm");
                Servo arm2 = hardwareMap.servo.get("arm2");

                arm1.setDirection(Servo.Direction.REVERSE);


                Servo claw = hardwareMap.servo.get("claw");

                Servo wrist = hardwareMap.servo.get("wrist");
                telemetry.addData("Slide1 Position", slide1.getCurrentPosition());
                telemetry.addData("Slide2 Position", slide1.getCurrentPosition());
                telemetry.update();

                wrist.setPosition(0.91);

                claw.setPosition(0.3);


                return false;
            }
        }


        public Action DepositPosition(int i) {
            return new DepositPosition(i);
        }

        public Action OpenClaw() {
            return new OpenClaw();

        }
        public Action CloseClaw() {
            return new CloseClaw();

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
                .strafeTo(new Vector2d(11,72))
                .strafeTo(new Vector2d(36,72))
                .waitSeconds(0.4)
                .build();

        ParkZone1 = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(11,72))
                .strafeTo(new Vector2d(11,60))
                .strafeTo(new Vector2d(36,60))
                .build();

        ParkZone2 = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(11,72))
                .strafeTo(new Vector2d(11,36))
                .strafeTo(new Vector2d(36,36))
                .build();

        ParkZone3 = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(11,72))
                .strafeTo(new Vector2d(11,12))
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



                        lift.DepositPosition(440)

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