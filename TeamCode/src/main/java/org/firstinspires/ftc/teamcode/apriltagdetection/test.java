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
public class test extends LinearOpMode {
    public class Lift {
        public class DepositPosition implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {



                DcMotorEx slide1 = hardwareMap.get(DcMotorEx.class, "slide1");
                DcMotorEx slide2 = hardwareMap.get(DcMotorEx.class, "slide2");

                slide1.setDirection(DcMotorSimple.Direction.REVERSE);
                slide2.setDirection(DcMotorSimple.Direction.FORWARD);

                // Ensures that when no power is set on the motors they will hold their position
                slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                waitForStart();

                slide2.setTargetPosition(200);
                sleep(5000);
                telemetry.addData("current pos", slide2.getCurrentPosition());
                telemetry.update();

                while(opModeIsActive()){sleep(20);}





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





