package org.firstinspires.ftc.teamcode.apriltagdetection;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class RobotCalibrator {
    private int testTime = 100;
    private double testPower = 0.5;
    public double powerCoeff = 0;
    private OpMode opmode;

    DcMotorEx slide1;
    DcMotorEx slide2;

    public RobotCalibrator(OpMode opmode_in){
        opmode = opmode_in;
    }
    public void calibrateSlides(){
        int testTime = 100;

        double startPos = slide2.getCurrentPosition();



        double endPos = slide2.getCurrentPosition();

        // speed in ticks per ms
        double testSpeed = (endPos - startPos) / testTime;

        double powerCoefficient = 0.5 / testSpeed;



    }
}
