package org.firstinspires.ftc.teamcode.apriltagdetection;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class Test extends LinearOpMode {
    public int target = 0;

    double Kp = 0.015;
    public class Lift {

        DcMotorEx slide1 = hardwareMap.get(DcMotorEx.class, "slide1");
        DcMotorEx slide2 = hardwareMap.get(DcMotorEx.class, "slide2");
        Servo arm1 = hardwareMap.servo.get("arm");
        Servo arm2 = hardwareMap.servo.get("arm2");

        Servo wrist = hardwareMap.servo.get("wrist");

        Servo claw = hardwareMap.servo.get("claw");


        public class setTarget implements Action {
            public setTarget(int i) {
                target = i;
            }

            public boolean run(@NonNull TelemetryPacket telemetryPacket) throws NullPointerException {
                return false;
            }
        }
        public class Init implements Action{
            public boolean run(@NonNull TelemetryPacket telemetryPacket) throws NullPointerException {
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
        public class GlobalPID implements Action {

            public boolean run(@NonNull TelemetryPacket telemetryPacket) throws NullPointerException {

                int error1 = target - slide1.getCurrentPosition();
                slide1.setPower(-error1 * Kp);
                int error2 = target - slide2.getCurrentPosition();
                slide2.setPower(error2 * Kp);
                telemetry.addData("slide1 pos", slide1.getCurrentPosition());
                telemetry.addData("slide2 pos", slide2.getCurrentPosition());
                telemetry.update();
          
                return true;
            }
        }

        public Action setTarget(int i) {return new setTarget(i);}
        public Action globalPID() {return new GlobalPID();}
        public Action initialize(){return new Init();}
    }

    public void runOpMode() {
        waitForStart();
        Lift lift = new Lift();

        Actions.runBlocking(new SequentialAction(
                        lift.initialize(),
                        lift.setTarget(440),
                        lift.globalPID(),
                        lift.setTarget(0)
                )
        );

        while (opModeIsActive()) {
            sleep(20);
        }
    }
}






