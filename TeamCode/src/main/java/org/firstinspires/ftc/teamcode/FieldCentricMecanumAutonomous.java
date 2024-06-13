package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*
 *  This OpMode illustrates the concept of driving an autonomous path based on Gyro (IMU) heading and encoder counts.
 *  The code is structured as a LinearOpMode
 *
 *  The path to be followed by the robot is built from a series of drive, turn or pause steps.
 *  Each step on the path is defined by a single function call, and these can be strung together in any order.
 *
 *  The code REQUIRES that you have encoders on the drive motors, otherwise you should use: RobotAutoDriveByTime;
 *
 *  This code uses the Universal IMU interface so it will work with either the BNO055, or BHI260 IMU.
 *  To run as written, the Control/Expansion hub should be mounted horizontally on a flat part of the robot chassis.
 *  The REV Logo should be facing UP, and the USB port should be facing forward.
 *  If this is not the configuration of your REV Control Hub, then the code should be modified to reflect the correct orientation.
 *
 *  This sample requires that the drive Motors have been configured with names : left_drive and right_drive.
 *  It also requires that a positive power command moves both motors forward, and causes the encoders to count UP.
 *  So please verify that both of your motors move the robot forward on the first move.  If not, make the required correction.
 *  See the beginning of runOpMode() to set the FORWARD/REVERSE option for each motor.
 *
 *  This code uses RUN_TO_POSITION mode for driving straight, and RUN_USING_ENCODER mode for turning and holding.
 *  Note: This code implements the requirement of calling setTargetPosition() at least once before switching to RUN_TO_POSITION mode.
 *
 *  Notes:
 *
 *  All angles are referenced to the coordinate-frame that is set whenever resetHeading() is called.
 *  In this sample, the heading is reset when the Start button is touched on the Driver station.
 *  Note: It would be possible to reset the heading after each move, but this would accumulate steering errors.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clockwise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  https://ftc-docs.firstinspires.org/field-coordinate-system
 *
 *  Control Approach.
 *
 *  To reach, or maintain a required heading, this code implements a basic Proportional Controller where:
 *
 *      Steering power = Heading Error * Proportional Gain.
 *
 *      "Heading Error" is calculated by taking the difference between the desired heading and the actual heading,
 *      and then "normalizing" it by converting it to a value in the +/- 180 degree range.
 *
 *      "Proportional Gain" is a constant that YOU choose to set the "strength" of the steering response.
 *
 *  Use Android Studio to Copy this Class, and Paste it into your "TeamCode" folder with a new name.
 *  Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous
public class FieldCentricMecanumAutonomous extends LinearOpMode {
    // Initialize default servo positions
    double arm1pos = 0.02;
    double arm2pos = 0.02;
    double wristpos= 0.91;

    final double MOTOR_PPR = 145.1; // aka ticks per rotation
    final double TICKS_PER_CM = (int) Math.round(145.1 / 12);

    // Maps motors and servos
    DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
    DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
    DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
    DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

    Servo arm1 = hardwareMap.servo.get("arm");
    Servo arm2 = hardwareMap.servo.get("arm2");

    Servo claw = hardwareMap.servo.get("claw");

    Servo wrist = hardwareMap.servo.get("wrist");

    DcMotorEx slide1 = hardwareMap.get(DcMotorEx.class,"slide1");
    DcMotorEx slide2 = hardwareMap.get(DcMotorEx.class,"slide2");
    // Retrieves the IMU from the hardware map
    IMU imu = hardwareMap.get(IMU.class, "imu");

    @Override
    public void runOpMode(){

        // Orients motors to allow for forward movement
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Ensures that when no power is set on the motors they will hold their position
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Our robots logo direction and usb direction
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));

        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        // Initalize slides
        slide1.setDirection(DcMotorSimple.Direction.REVERSE);
        slide2.setDirection(DcMotorSimple.Direction.FORWARD);

        // Ensures that when no power is set on the motors they will hold their position
        slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm1.setDirection(Servo.Direction.REVERSE);
    }

    //vv just arick messing around lol vv
    private void moveRobot(double moveAngle){
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double relativeAngle = botHeading-moveAngle;
        if(relativeAngle==Math.PI/2){
            frontLeftMotor.setPower(1);
            frontRightMotor.setPower(-1);
            backRightMotor.setPower(1);
            backLeftMotor.setPower(-1);
        }
        else if(relativeAngle==-Math.PI/2){
            frontLeftMotor.setPower(-1);
            frontRightMotor.setPower(1);
            backRightMotor.setPower(-1);
            backLeftMotor.setPower(1);
        }
        else {
            double ratio = Math.tan(relativeAngle);
            double xPow;
            double yPow;
            if (Math.abs(ratio) <= 1) {
                yPow = 1*ratio/Math.abs(ratio);
                xPow = yPow * ratio;
            } else {
                xPow = 1*ratio/Math.abs(ratio);
                yPow = xPow * (1 / ratio);
            }
            if (xPow > 0 && yPow > 0) {
                frontLeftMotor.setPower(1);
                backRightMotor.setPower(1);
                if (xPow != 1) {
                    frontRightMotor.setPower(1 - xPow);
                    backLeftMotor.setPower(1 - xPow);
                } else {
                    frontRightMotor.setPower(yPow - 1);
                    backLeftMotor.setPower(yPow - 1);
                }
            } else if (xPow < 0 && yPow > 0) {
                frontRightMotor.setPower(1);
                backLeftMotor.setPower(1);
                if (xPow != -1) {
                    frontLeftMotor.setPower(xPow + 1);
                    backRightMotor.setPower(xPow + 1);
                } else {
                    frontLeftMotor.setPower(yPow - 1);
                    backRightMotor.setPower(yPow - 1);
                }
            } else if (xPow < 0 && yPow < 0) {
                frontLeftMotor.setPower(-1);
                backRightMotor.setPower(-1);
                if (xPow != -1) {
                    frontRightMotor.setPower(-xPow - 1);
                    backLeftMotor.setPower(-xPow - 1);
                } else {
                    frontRightMotor.setPower(yPow + 1);
                    backLeftMotor.setPower(yPow + 1);
                }
            } else if (xPow > 0 && yPow < 0) {
                frontRightMotor.setPower(-1);
                backLeftMotor.setPower(-1);
                if (xPow != 1) {
                    frontLeftMotor.setPower(xPow - 1);
                    backRightMotor.setPower(xPow - 1);
                } else {
                    frontLeftMotor.setPower(yPow + 1);
                    backRightMotor.setPower(yPow + 1);
                }
            }
        }
    }
}