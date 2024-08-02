package org.firstinspires.ftc.teamcode.apriltagdetection;

import static java.lang.Thread.sleep;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class Tets extends OpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    static final double FEET_PER_METER = 3.28084;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double tagsize = 0.166;
    int leftTag = 17;
    int middleTag = 18;
    int rightTag = 19;
    AprilTagDetection tagOfInterest = null;
    public int target = 0;
    double Kp = 0.02;
    int iterations = 0;
    double arm1pos = 0.02;
    double arm2pos = 0.02;
    double clawpos = 0.3;
    double wristpos = 0.91;
    private boolean ran = false;
    public Lift lift;
    public boolean tagFound = false;
    public Action DriveInitialDeposit;
    public Action ParkZone1;
    public Action ParkZone2;
    public Action ParkZone3;
    public Action parkingZone;

    public class Lift {
        DcMotorEx slide1 = hardwareMap.get(DcMotorEx.class, "slide1");
        DcMotorEx slide2 = hardwareMap.get(DcMotorEx.class, "slide2");
        Servo arm1 = hardwareMap.servo.get("arm");
        Servo arm2 = hardwareMap.servo.get("arm2");
        Servo wrist = hardwareMap.servo.get("wrist");
        Servo claw = hardwareMap.servo.get("claw");

        public class SetSlidesTarget implements Action {
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                return false;
            }

            public SetSlidesTarget(int e) {
                target = e;
            }
        }

        public class OpenClaw implements Action {
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                clawpos = 0;
                return false;
            }
        }

        public class ArmsDown implements Action {
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                arm1pos = 0.02;
                arm2pos = 0.02;
                wristpos = 0.91;
                clawpos = 0.3;
                return false;
            }
        }

        public class CloseClaw implements Action {
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                clawpos = 0.3;
                return false;
            }
        }

        public class ArmsUp implements Action {
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                arm1pos = 0.96;
                arm2pos = 0.96;
                wristpos = 0.3;
                clawpos = 0.3;
                return false;
            }
        }

        public class Init implements Action {
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                arm1.setDirection(Servo.Direction.REVERSE);
                arm2.setDirection(Servo.Direction.FORWARD);

                slide1.setDirection(DcMotorSimple.Direction.REVERSE);
                slide2.setDirection(DcMotorSimple.Direction.FORWARD);

                slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                arm1.setPosition(0.02);
                arm2.setPosition(0.02);
                wrist.setPosition(0.91);
                claw.setPosition(0.3);

                return false;
            }
        }

        public void GlobalPID() {

            int error1 = target - slide1.getCurrentPosition();
            slide1.setPower(-error1 * Kp);
            int error2 = target - slide2.getCurrentPosition();
            slide2.setPower(error2 * Kp);

            arm1.setPosition(arm1pos);
            arm2.setPosition(arm2pos);
            claw.setPosition(clawpos);
            wrist.setPosition(wristpos);

            telemetry.addData("slide1 pos", slide1.getCurrentPosition());
            telemetry.addData("slide2 pos", slide2.getCurrentPosition());
            telemetry.addData("iterations", iterations++);
            telemetry.update();
        }

        public Action initialize() {
            return new Init();
        }

        public Action setSlidesTarget(int e) {
            return new SetSlidesTarget(e);
        }

        public Action openClaw() {
            return new OpenClaw();
        }

        public Action armsDown() {
            return new ArmsDown();
        }

        public Action closeClaw() {
            return new CloseClaw();
        }

        public Action armsUp() {
            return new ArmsUp();
        }

        public void globalPidUpdate() {
            GlobalPID();
        }
    }


    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(11, 36, Math.toRadians(0)));

        Action DriveInitialDeposit = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(11, 72))
                .strafeTo(new Vector2d(36, 72))
                .build();

        Action ParkZone1 = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(11, 72))
                .strafeTo(new Vector2d(11, 60))
                .strafeTo(new Vector2d(36, 60))
                .build();

        Action ParkZone2 = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(11, 72))
                .strafeTo(new Vector2d(11, 36))
                .strafeTo(new Vector2d(36, 36))
                .build();

        Action ParkZone3 = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(11, 72))
                .strafeTo(new Vector2d(11, 12))
                .strafeTo(new Vector2d(36, 12))
                .build();

        tagFound = false;

        lift = new Lift();
        Actions.runBlocking(lift.initialize());
    }

    @Override
    public void start() {

        while (!tagFound) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == leftTag || tag.id == middleTag || tag.id == rightTag) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                }

            }

            telemetry.update();
        }


        /* Update the telemetry */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        parkingZone = null;

        if (tagOfInterest == null) {

            // Runs autonomous and parks in zone 2
            // Runs if camera does not detect april tag
            parkingZone = ParkZone2;
        }
        if (tagOfInterest.id == leftTag) {

            parkingZone = ParkZone1;
        } else if (tagOfInterest.id == middleTag) {

            parkingZone = ParkZone2;
        } else if (tagOfInterest.id == rightTag) {

            parkingZone = ParkZone3;
        }


    }


    @Override
    public void loop() {
        lift.globalPidUpdate();
        if (!ran) {
            ran = true;
            Actions.runBlocking(
                    new SequentialAction(
                            DriveInitialDeposit,
                            new SleepAction(2),
                            lift.setSlidesTarget(440),
                            lift.armsUp(),
                            new SleepAction(2),
                            lift.openClaw(),
                            new SleepAction(2),
                            lift.armsDown(),
                            new SleepAction(0.75),
                            lift.setSlidesTarget(0),
                            parkingZone
                    )
            );
        }
    }
}