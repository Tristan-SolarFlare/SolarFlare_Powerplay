package org.firstinspires.ftc.teamcode.apriltagdetection;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class SlideConfig extends LinearOpMode {

    public class Lift {
        public class DepositPosition implements Action {
            DcMotorEx slide1;
            DcMotorEx slide2;

            public double errorToTime(int error){
                return error * 100;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                slide1 = hardwareMap.get(DcMotorEx.class, "slide1");
                slide2 = hardwareMap.get(DcMotorEx.class, "slide2");

                slide1.setDirection(DcMotorSimple.Direction.REVERSE);
                slide2.setDirection(DcMotorSimple.Direction.FORWARD);

                // Ensures that when no power is set on the motors they will hold their position
                slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


                //int target = 440;
                int testTime = 100;

                double startPos = slide2.getCurrentPosition();

                slide2.setPower(0.5);
                sleep(testTime);
                slide2.setPower(0);

                double endPos = slide2.getCurrentPosition();

                // speed in ticks per ms
                double testSpeed = (endPos - startPos) / testTime;

                double powerCoefficient = 0.5 / testSpeed;

                int target = 200;





                telemetry.addData("start pos", startPos);
                telemetry.addData("end pos", endPos);
                telemetry.addData("speed", testSpeed);
                telemetry.addData("Power Coefficient", powerCoefficient);
                telemetry.update();

                waitForStart();

                return false;
            }
        }

        public Action DepositPosition() {
            return new DepositPosition();
        }


    }
    public void runOpMode() {

        Lift lift = new Lift();
        Actions.runBlocking(
                lift.DepositPosition()
        );


    }
}





