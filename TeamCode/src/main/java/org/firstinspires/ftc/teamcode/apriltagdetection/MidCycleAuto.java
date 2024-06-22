/* https://drive.google.com/file/d/1GAEcuIJ_Pu-IoopSYptUfXAdABRY6eKw/view
    upload above file to control hub. Drag into "FIRST" folder.
 */

package org.firstinspires.ftc.teamcode.apriltagdetection;

import androidx.annotation.NonNull;
import androidx.appcompat.widget.VectorEnabledTintResources;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
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
        private DcMotorEx slide1;
        private DcMotorEx slide2;

        public Lift(HardwareMap hardwareMap) {
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
        }
        // mid=120
        // topcone=150
        // second=120
        // third=85
        // fourth=40
        // last=0

        public class TopCone implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                double Kp=0.015;
                while (slide1.getCurrentPosition() <150 && slide2.getCurrentPosition() <150){
                    double slide1power;
                    // Calculates amount of ticks off slide1 is from target
                    double error1=-(175-slide1.getCurrentPosition()); // Error is negative because slide1 needs to reverse direction

                    // If error1 is greater than 80, correct by faster speed, else correct by usual speed
                    if ((Math.abs(error1)>80)){
                        slide1power = (0.75*error1);
                    }
                    else {
                        slide1power = (Kp*error1);
                    }
                    slide1.setPower(slide1power);

                    // We need a new slide2 power that will correct for error
                    double slide2power;

                    // Calculates amount of ticks off slide2 is from target
                    double error2=175-slide2.getCurrentPosition();

                    // If error1 is greater than 80, correct by faster speed, else correct by usual speed
                    if ((Math.abs(error2)>80)){
                        slide2power = 0.75*error2;
                    }
                    else {
                        slide2power = Kp*error2;
                    }
                    slide2.setPower(slide2power);
                }

                return true;
            }
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


        Action parkingZone1;
        Action parkingZone2;
        Action parkingZone3;

        //set staring position, unit is inches
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(11, 36, Math.toRadians(0)));

        parkingZone1 = drive.actionBuilder(drive.pose)
                .turn(Math.toRadians(90))

                .strafeTo(new Vector2d(45, 36))
                .waitSeconds(0.4)
                .strafeTo(new Vector2d(57, 36))
                .strafeTo(new Vector2d(56.5, 12))
                .waitSeconds(0.2)
                .strafeToLinearHeading(new Vector2d(55, 38), Math.toRadians(120))
                .waitSeconds(0.4)

                .strafeToLinearHeading(new Vector2d(56, 12), Math.toRadians(90))


                .waitSeconds(0.2)
                .strafeToLinearHeading(new Vector2d(55, 38), Math.toRadians(120))


                .waitSeconds(0.4)
                .strafeToLinearHeading(new Vector2d(56, 12), Math.toRadians(90))

                .waitSeconds(0.2)

                .strafeToLinearHeading(new Vector2d(55, 38), Math.toRadians(120))


                .waitSeconds(0.4)

                .strafeToLinearHeading(new Vector2d(55, 12), Math.toRadians(90))


                .waitSeconds(0.2)

                .strafeToLinearHeading(new Vector2d(55, 38), Math.toRadians(120))


                .waitSeconds(0.4)
                .strafeToLinearHeading(new Vector2d(55, 12), Math.toRadians(90))


                .waitSeconds(0.2)

                .strafeToLinearHeading(new Vector2d(55,38),Math.toRadians(120))


                .waitSeconds(0.4)
                .turn(Math.toRadians(-120))

                .strafeTo(new Vector2d(36,36))
                .strafeTo(new Vector2d(36,60))

                .build();
        parkingZone2 = drive.actionBuilder (drive.pose)
                .turn(Math.toRadians(90))

                .strafeTo(new Vector2d(45, 36))
                .waitSeconds(0.4)
                .strafeTo(new Vector2d(57, 36))
                .strafeTo(new Vector2d(56.5, 12))
                .waitSeconds(0.2)
                .strafeToLinearHeading(new Vector2d(55, 38), Math.toRadians(120))
                .waitSeconds(0.4)

                .strafeToLinearHeading(new Vector2d(56, 12), Math.toRadians(90))


                .waitSeconds(0.2)
                .strafeToLinearHeading(new Vector2d(55, 38), Math.toRadians(120))


                .waitSeconds(0.4)
                .strafeToLinearHeading(new Vector2d(56, 12), Math.toRadians(90))

                .waitSeconds(0.2)

                .strafeToLinearHeading(new Vector2d(55, 38), Math.toRadians(120))


                .waitSeconds(0.4)

                .strafeToLinearHeading(new Vector2d(55, 12), Math.toRadians(90))


                .waitSeconds(0.2)

                .strafeToLinearHeading(new Vector2d(55, 38), Math.toRadians(120))


                .waitSeconds(0.4)
                .strafeToLinearHeading(new Vector2d(55, 12), Math.toRadians(90))


                .waitSeconds(0.2)

                .strafeToLinearHeading(new Vector2d(55,38),Math.toRadians(120))


                .waitSeconds(0.4)
                .turn(Math.toRadians(-120))

                .build();
        parkingZone3 = drive.actionBuilder(drive.pose)
                .turn(Math.toRadians(90))

                .strafeTo(new Vector2d(45, 36))
                .waitSeconds(0.4)
                .strafeTo(new Vector2d(57, 36))
                .strafeTo(new Vector2d(56.5, 12))
                .waitSeconds(0.2)
                .strafeToLinearHeading(new Vector2d(55, 38), Math.toRadians(120))
                .waitSeconds(0.4)

                .strafeToLinearHeading(new Vector2d(56, 12), Math.toRadians(90))


                .waitSeconds(0.2)
                .strafeToLinearHeading(new Vector2d(55, 38), Math.toRadians(120))


                .waitSeconds(0.4)
                .strafeToLinearHeading(new Vector2d(56, 12), Math.toRadians(90))

                .waitSeconds(0.2)

                .strafeToLinearHeading(new Vector2d(55, 38), Math.toRadians(120))


                .waitSeconds(0.4)

                .strafeToLinearHeading(new Vector2d(55, 12), Math.toRadians(90))


                .waitSeconds(0.2)

                .strafeToLinearHeading(new Vector2d(55, 38), Math.toRadians(120))


                .waitSeconds(0.4)
                .strafeToLinearHeading(new Vector2d(55, 12), Math.toRadians(90))


                .waitSeconds(0.2)

                .strafeToLinearHeading(new Vector2d(55,38),Math.toRadians(120))


                .waitSeconds(0.4)
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
            trajectoryActionChosen = parkingZone2;
        }
        if(tagOfInterest.id == leftTag){
            // 
            trajectoryActionChosen = parkingZone1;
        }
        else if (tagOfInterest.id == middleTag){
            trajectoryActionChosen = parkingZone2;
        }
        else if (tagOfInterest.id == rightTag){
            trajectoryActionChosen = parkingZone3;
        }
        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen
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