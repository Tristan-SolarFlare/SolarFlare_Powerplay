package org.firstinspires.ftc.teamcode.apriltagdetection;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
@Autonomous
public class ObjectDetectionCV extends LinearOpMode{
    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;
    MecanumDrive drive;
    OpenCvCamera camera;

    Servo claw;
    int location;
    ArrayList<Integer> junction=new ArrayList<>();
    boolean arrived=false;

    public class TurnToCone implements Action {
        public boolean run(@NonNull TelemetryPacket packet){
            if(location==3 || location==0){
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .turn(Math.toRadians(10)).build()
                );
            }
            else if(location==1){
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .turn(Math.toRadians(-10)).build()
                );
            }
            return !arrived;
        }
    }

    public class MoveToCone implements Action {
        public boolean run(@NonNull TelemetryPacket packet){
            if(location==2 && !arrived){
                leftFront.setPower(-1);
                leftBack.setPower(-1);
                rightFront.setPower(-1);
                rightBack.setPower(-1);
            }
            else{
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightFront.setPower(0);
                rightBack.setPower(0);
            }
            return !arrived;
        }
    }

    public class DodgeJunction implements Action {
        public boolean run(@NonNull TelemetryPacket packet){
            if(!junction.isEmpty()) {
                if ((junction.get(0) + junction.get(junction.size() - 1)) / 2 <= 8) {
                    leftFront.setPower(1);
                    rightFront.setPower(-1);
                    leftBack.setPower(1);
                    rightBack.setPower(-1);

                    ;
                } else {
                    leftFront.setPower(-1);
                    rightFront.setPower(1);
                    leftBack.setPower(-1);
                    rightBack.setPower(1);
                }
            }
            return !arrived;
        }
    }

    public class GrabCone implements Action {
        public boolean run(@NonNull TelemetryPacket packet){
            Actions.runBlocking(drive.actionBuilder(drive.pose).turn(Math.toRadians(180)).build());
            leftFront.setPower(1);
            leftBack.setPower(1);
            rightFront.setPower(1);
            rightBack.setPower(1);
            sleep(400);
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);
            claw.setPosition(0);
            return false;
        }
    }
    public void runOpMode(){
        waitForStart();
        claw=hardwareMap.servo.get("claw");
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive=new MecanumDrive(hardwareMap, new Pose2d(11, 36, Math.toRadians(0)));
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");

        claw.setPosition(1);
        sleep(200);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        camera.setPipeline(new ConeDetection());

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
        Actions.runBlocking(new SequentialAction(
                drive.actionBuilder(drive.pose).turn(Math.toRadians(-90)).build(),
                new ParallelAction(new TurnToCone(),new MoveToCone(),new DodgeJunction()),
                new GrabCone()
                )
        );

    }
    public class ConeDetection extends OpenCvPipeline{

        public Scalar lower = new Scalar(83,107,184);
        public Scalar upper = new Scalar(108,110,176);
        public Scalar lowjunction = new Scalar(145,157,85);
        public Scalar upjunction = new Scalar(192,155,86);

        public Mat processFrame(Mat input){
            Mat junctions= new Mat();
            Imgproc.cvtColor(input,input,Imgproc.COLOR_RGB2YCrCb);
            input.copyTo(junctions);

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
                else{
                    arrived=false;
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

            Rect rectsect;

            ArrayList<Integer> scan=new ArrayList<>();
            boolean foundJunction = false;
            for(int i=1;i<16;i++){
                rectsect = new Rect(53*(i-1)+1,1,53*i,448);
                Mat section = input.submat(rectsect);
                Core.inRange(section, lowjunction, upjunction, section);
                double avg = Core.mean(section).val[0];
                if(avg>0){
                    scan.add(i);
                    if(scan.size()>=3){
                        foundJunction=true;
                        junction=scan;
                        break;
                    }
                }
                else{
                    scan.clear();
                }
            }
            if(!foundJunction){
                junction=new ArrayList<>();
            }
            telemetry.addData("Junction", junction.get(0).toString()+junction.get(junction.size()-1).toString());
            return input;

        }
    }
}
