/* https://drive.google.com/file/d/1GAEcuIJ_Pu-IoopSYptUfXAdABRY6eKw/view
    upload above file to control hub. Drag into "FIRST" folder.
 */

package org.firstinspires.ftc.teamcode.apriltagdetection;

import androidx.appcompat.widget.VectorEnabledTintResources;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

    @Override
    public void runOpMode()
    {
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

            // Runs autonomous and parks in zone 2
            // Runs if camera does not detect april tag
            trajectoryActionChosen = parkingZone2;
        }
        if(tagOfInterest.id == leftTag){

            // Runs autonomous and parks in zone 1
            trajectoryActionChosen = parkingZone1;
        }
        else if (tagOfInterest.id == middleTag){

            // Runs autonomous and parks in zone 2
            trajectoryActionChosen = parkingZone2;
        }
        else if (tagOfInterest.id == rightTag){

            // Runs autonomous and parks in zone 3
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