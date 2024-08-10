package org.firstinspires.ftc.teamcode.apriltagdetection;

import static java.lang.Math.max;

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
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
@Autonomous
public class SimpleObjectDetectionCV extends LinearOpMode{

    OpenCvCamera camera;

    int location;
    public void runOpMode(){

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        camera.setPipeline(new DetectionPipeline2());

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode){

            }
        });
        waitForStart();
    }

    public class DetectionPipeline1 extends OpenCvPipeline{
        public Mat processFrame(Mat input){

            Imgproc.cvtColor(input,input,Imgproc.COLOR_RGB2YCrCb);
            Rect left = new Rect(1,1,267,400);
            Rect middle = new Rect(267,1,266,400);
            Rect right = new Rect(533,1,267,400);

            Mat leftCrop=input.submat(left);
            Mat middleCrop=input.submat(middle);
            Mat rightCrop=input.submat(right);

            Core.extractChannel(leftCrop, leftCrop, 3);
            Core.extractChannel(middleCrop, middleCrop, 3);
            Core.extractChannel(rightCrop, rightCrop, 3);

            double leftavg = (int)Core.mean(leftCrop).val[0];
            double midavg = (int)Core.mean(middleCrop).val[0];
            double rightavg = (int)Core.mean(rightCrop).val[0];

            if (leftavg>midavg) {
                if(leftavg>rightavg){
                    location=1;
                }
                else{
                    location=3;
                }
            }
            else{
                if(midavg>rightavg){
                    location=2;
                }
                else{
                    location=3;
                }
            }
            Core.extractChannel(input,input,3);
            telemetry.addData("Location",location);
            return input;
        }
    }
    public class DetectionPipeline2 extends OpenCvPipeline{

        public Scalar lower = new Scalar(70,87,164);
        public Scalar upper = new Scalar(108,110,176);
        public Mat processFrame(Mat input){

            Imgproc.cvtColor(input,input,Imgproc.COLOR_RGB2YCrCb);
            Rect left = new Rect(1,1,267,400);
            Rect middle = new Rect(267,1,266,400);
            Rect right = new Rect(533,1,267,400);

            Mat leftCrop=input.submat(left);
            Mat middleCrop=input.submat(middle);
            Mat rightCrop=input.submat(right);

            Core.inRange(leftCrop, lower, upper, leftCrop);
            Core.inRange(middleCrop, lower, upper, middleCrop);
            Core.inRange(rightCrop, lower, upper, rightCrop);

            double leftavg = (int)Core.mean(leftCrop).val[0];
            double midavg = (int)Core.mean(middleCrop).val[0];
            double rightavg = (int)Core.mean(rightCrop).val[0];

            if (leftavg>midavg) {
                if(leftavg>rightavg){
                    location=1;
                }
                else{
                    location=3;
                }
            }
            else{
                if(midavg>rightavg){
                    location=2;
                }
                else{
                    location=3;
                }
            }
            Core.inRange(input, lower, upper, input);
            telemetry.addData("Location",location);
            return input;

        }
    }

}
