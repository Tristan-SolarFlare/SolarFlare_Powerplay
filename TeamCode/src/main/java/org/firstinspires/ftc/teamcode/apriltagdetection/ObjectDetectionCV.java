package org.firstinspires.ftc.teamcode.apriltagdetection;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous
public class ObjectDetectionCV extends LinearOpMode{

    OpenCvCamera camera;

    int location;
    boolean arrived=false;
    public void runOpMode(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        camera.setPipeline(new DetectionPipeline());

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode){

            }
        });
    }
    public class DetectionPipeline extends OpenCvPipeline{

        public Scalar lower = new Scalar(83,107,184);
        public Scalar upper = new Scalar(108,110,176);
        public Mat processFrame(Mat input){

            Imgproc.cvtColor(input,input,Imgproc.COLOR_RGB2YCrCb);
            Rect left = new Rect(1,1,400,448);
            Rect right = new Rect(400,1,400,448);
            Rect top = new Rect(1,348,800,100);

            Mat leftCrop=input.submat(left);
            Mat rightCrop=input.submat(right);
            Mat topCrop=input.submat(top);

            Core.inRange(leftCrop, lower, upper, leftCrop);
            Core.inRange(rightCrop, lower, upper, rightCrop);
            Core.inRange(topCrop, lower, upper, topCrop);
            Core.inRange(input, lower, upper, input);

            double leftavg = Core.mean(leftCrop).val[0];
            double rightavg = Core.mean(rightCrop).val[0];
            double topavg= Core.mean(topCrop).val[0];

            if (Math.abs(leftavg-rightavg)<1) {
                location=2;
                if(topavg>0){
                    arrived=true;
                }
            }
            else if(leftavg>0){
                location=1;
            }
            else if(rightavg>0){
                location=3;
            }
            else{
                location=0;
            }
            telemetry.addData("Location",location);
            telemetry.addData("left",leftavg);
            telemetry.addData("right",rightavg);
            telemetry.addData("Arrived", arrived);
            return input;

        }
    }

}
