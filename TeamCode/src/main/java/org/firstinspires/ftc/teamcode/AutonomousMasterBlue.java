/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.internal.for_camera_opmodes.LinearOpModeCamera;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Autonomous: Blue")
public class AutonomousMasterBlue extends LinearOpModeCamera {

    /* Declare OpMode members. */
    HardwareRobot robot = new HardwareRobot();
    private ElapsedTime runtime = new ElapsedTime();

    static final double     countsPerMotorRev    = 1120 ;
    static final double     robotDiameterCm    = 35.0 ;
    static final double     wheelDiameterCm   = 10.0 ;
    static final double     countsPerCm    = countsPerMotorRev / (wheelDiameterCm * Math.PI);

    static final double     DRIVE_SPEED = 0.7;

    static final int ds2 = 8;
    Orientation iniAngle;
    Orientation angle;

    @Override
    public void runOpMode() {

        if (isCameraAvailable()) {

            startCamera();

            robot.init(hardwareMap);

            telemetry.addData("Mode", "waiting for start");
            telemetry.addData("imu calib status", robot.imu.getCalibrationStatus().toString());
            telemetry.update();

            resetEncoders();

            robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.autoArmDown.setPosition(0.65);
            robot.autoArmUp.setPosition(0.7);

            robot.backLeft.setPosition(0.5);
            robot.backRight.setPosition(0.5);

            robot.dipper.setPosition(0.2);
            robot.picker.setPosition(0.7);

            // Wait for the game to start (driver presses PLAY)
            waitForStart();

            robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 500);
            iniAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            angle = iniAngle;

            float posAvg = 0;
            for (int i=0; i < 3; i++) {
                int position = analyze();
                posAvg += position;
            }
            int finalPos = Math.round(posAvg / 3);
            stopCamera();
            telemetry.addData("Place:", finalPos);
            telemetry.update();

            if(finalPos == 2) {
                DiagonalDrive(-1, 0.478, 135, 1800);
                robot.autoArmDown.setPosition(0);
                sleep(450);
                robot.autoArmUp.setPosition(0);
                sleep(500);
                robot.autoArmDown.setPosition(0.65);
                sleep(300);

                DiagonalDrive(1, -1, 0, 225);
                calibrateAngle(0.2, -0.1);                  //ANGLE
                resetEncoders();
                straightDrive(1,-175,8);
                calibrateAngle(0.2, 0);                    //ANGLE
                DiagonalDrive(-1, 1, 0, 250);

                robot.autoArmDown.setPosition(0.2);
                sleep(200);
                robot.autoArmUp.setPosition(0.6);
                sleep(200);
                robot.autoArmDown.setPosition(0.6);
                DiagonalDrive(1, -1, 0, 245);
                calibrateAngle(0.2, 0);                  //ANGLE

                resetEncoders();
                straightDrive(1,240,7);

                DiagonalDrive(-0.75, 0.75, 125, 2000);
                sleep(100);
                robot.autoArmDown.setPosition(0);
                sleep(450);
                robot.autoArmUp.setPosition(0);
                sleep(400);
                robot.autoArmDown.setPosition(0.8);
                sleep(350);

                DiagonalDrive(1, -1, 0, 225);
                calibrateAngle(0.2, -0.1);                 //ANGLE
                resetEncoders();
                straightDrive(1,-230,6);
                calibrateAngle(0.2, 0);                    //ANGLE

                DiagonalDrive(-1, 1, 0, 260);
                robot.autoArmDown.setPosition(0.1);
                sleep(150);
                robot.autoArmUp.setPosition(0.6);
                sleep(250);
                robot.autoArmDown.setPosition(0.6);
                DiagonalDrive(1, -1, 0, 220);
                straightDrive(0.7,80,6);
            }
            else if(finalPos == 1) {
                DiagonalDrive(-0.8, 0.7, 120, 1700);
                robot.autoArmDown.setPosition(0);
                sleep(450);
                robot.autoArmUp.setPosition(0);
                sleep(500);
                robot.autoArmDown.setPosition(0.65);
                sleep(300);

                DiagonalDrive(1, -1, 0, 225);
                calibrateAngle(0.2, -0.1);                  //ANGLE
                resetEncoders();
                straightDrive(1,-210,7);
                calibrateAngle(0.2, 0);                    //ANGLE
                DiagonalDrive(-1, 1, 0, 255);

                robot.autoArmDown.setPosition(0.2);
                sleep(200);
                robot.autoArmUp.setPosition(0.6);
                sleep(200);
                robot.autoArmDown.setPosition(0.6);
                DiagonalDrive(1, -1, 0, 250);
                calibrateAngle(0.2, 0.1);                  //ANGLE

                resetEncoders();
                straightDrive(1,275,7);

                DiagonalDrive(-0.75, 0.75, 120, 2000);
                sleep(100);
                robot.autoArmDown.setPosition(0);
                sleep(450);
                robot.autoArmUp.setPosition(0);
                sleep(450);
                robot.autoArmDown.setPosition(0.8);
                sleep(350);

                DiagonalDrive(1, -1, 0, 225);
                calibrateAngle(0.2, 0.1);                 //ANGLE
                resetEncoders();
                straightDrive(1,-250,6);
                calibrateAngle(0.2, 0);                    //ANGLE

                DiagonalDrive(-1, 1, 0, 250);
                robot.autoArmDown.setPosition(0.1);
                sleep(150);
                robot.autoArmUp.setPosition(0.6);
                sleep(250);
                robot.autoArmDown.setPosition(0.6);
                DiagonalDrive(1, -1, 0, 185);
                straightDrive(0.7,80,6);


            }
            else if(finalPos == 0) {
                DiagonalDrive(-0.80, 1, 150, 1700);
                robot.autoArmDown.setPosition(0);
                sleep(400);
                robot.autoArmUp.setPosition(0);
                sleep(500);
                robot.autoArmDown.setPosition(0.65);
                sleep(300);

                DiagonalDrive(1, -1, 0, 225);
                calibrateAngle(0.2, 0.1);                  //ANGLE
                resetEncoders();
                straightDrive(1,-230,8);
                calibrateAngle(0.2, 0);                    //ANGLE
                DiagonalDrive(-1, 1, 0, 245);

                robot.autoArmDown.setPosition(0.2);
                sleep(200);
                robot.autoArmUp.setPosition(0.6);
                sleep(200);
                robot.autoArmDown.setPosition(0.6);
                DiagonalDrive(1, -1, 0, 240);
                calibrateAngle(0.2, 0.1);                  //ANGLE

                resetEncoders();
                straightDrive(1,275,8);

                DiagonalDrive(-0.75, 0.75, 120, 2000);
                sleep(100);
                robot.autoArmDown.setPosition(0);
                sleep(400);
                robot.autoArmUp.setPosition(0);
                sleep(450);
                robot.autoArmDown.setPosition(0.8);
                sleep(350);

                DiagonalDrive(1, -1, 0, 225);
                calibrateAngle(0.2, 0);                 //ANGLE
                resetEncoders();
                straightDrive(1,-275,6);
                calibrateAngle(0.2, 0);                    //ANGLE

                DiagonalDrive(-1, 1, 0, 265);
                robot.autoArmDown.setPosition(0.1);
                sleep(150);
                robot.autoArmUp.setPosition(0.6);
                sleep(250);
                robot.autoArmDown.setPosition(0.6);
                DiagonalDrive(1, -1, 0, 225);
                straightDrive(0.7,100,6);
            }


            telemetry.addData("Path", "Complete");
            telemetry.update();
        }
    }

    private void DiagonalDrive(double iPower, double kPower, double sensorMM, int timeoutMili) {
        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.leftFront.setPower(iPower);
        robot.rightBack.setPower(iPower);

        robot.rightFront.setPower(kPower);
        robot.leftBack.setPower(kPower);
        runtime.reset();

        while (opModeIsActive() &&
                (robot.sensorRange.getDistance(DistanceUnit.MM)*Math.cos(0.57) > sensorMM) &&
                runtime.milliseconds() < timeoutMili) {

        }
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.rightBack.setPower(0);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void calibrateAngle(double power, double targetAngle) {
        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        angle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        while(angle.firstAngle < targetAngle-0.1 || angle.firstAngle > targetAngle+0.1) {
            angle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            if(angle.firstAngle < targetAngle) {
                robot.rightFront.setPower(-power);
                robot.rightBack.setPower(-power);
                robot.leftFront.setPower(power);
                robot.leftBack.setPower(power);
            }
            if(angle.firstAngle > targetAngle) {
                robot.rightFront.setPower(power);
                robot.rightBack.setPower(power);
                robot.leftFront.setPower(-power);
                robot.leftBack.setPower(-power);
            }
            telemetry.addData("angle", angle.firstAngle);
            telemetry.update();
        }
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.rightBack.setPower(0);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    private void calibrateAnlgeEncoder(double power, double targetAngle, int timeoutS) {

        int robotRadius = 19;
        angle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double delta = angle.firstAngle - targetAngle;
        double deltaRad = delta * (Math.PI/180);

        int rotateAmount = 0;

        rotateAmount = (int)deltaRad * robotRadius;

        int targetLF = robot.leftFront.getCurrentPosition() + (int)(rotateAmount * countsPerCm);
        int targetRF = robot.rightFront.getCurrentPosition() + (int)(-rotateAmount * countsPerCm);
        int targetLB = robot.leftBack.getCurrentPosition() + (int)(rotateAmount * countsPerCm);
        int targetRB = robot.rightBack.getCurrentPosition() + (int)(-rotateAmount * countsPerCm);

        robot.leftFront.setTargetPosition(targetLF);
        robot.rightFront.setTargetPosition(-targetRF);
        robot.leftBack.setTargetPosition(targetLB);
        robot.rightBack.setTargetPosition(-targetRB);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();

        robot.leftFront.setPower(power);
        robot.rightFront.setPower(power);
        robot.leftBack.setPower(power);
        robot.rightBack.setPower(power);

        while (opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (robot.leftFront.isBusy() && robot.rightFront.isBusy() && robot.leftBack.isBusy() && robot.rightFront.isBusy())) {

        }

        // Stop all motion;
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.rightBack.setPower(0);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void straightDrive(double power, double x, double timeoutS) {

        int targetLF = robot.leftFront.getCurrentPosition() + (int)(x * countsPerCm);
        int targetRF = robot.rightFront.getCurrentPosition() + (int)(x * countsPerCm);
        int targetLB = robot.leftBack.getCurrentPosition() + (int)(x * countsPerCm);
        int targetRB = robot.rightBack.getCurrentPosition() + (int)(x * countsPerCm);

        robot.leftFront.setTargetPosition(targetLF);
        robot.rightFront.setTargetPosition(targetRF);
        robot.leftBack.setTargetPosition(targetLB);
        robot.rightBack.setTargetPosition(targetRB);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();

        robot.leftFront.setPower(power);
        robot.rightFront.setPower(power);
        robot.leftBack.setPower(power);
        robot.rightBack.setPower(power);

        while (opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (robot.leftFront.isBusy() && robot.rightFront.isBusy() && robot.leftBack.isBusy() && robot.rightFront.isBusy())) {

            // Display it for the driver.
                /*telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d",
                                            robot.leftFront.getCurrentPosition(),
                                            robot.rightFront.getCurrentPosition(),
                                            robot.leftBack.getCurrentPosition(),
                                            robot.rightBack.getCurrentPosition());*/
        }

        // Stop all motion;
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.rightBack.setPower(0);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    private void frontDrive(double power, double sensorMM, int timeoutMili) {
        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Sets power according to group
        robot.leftFront.setPower(-power);
        robot.rightBack.setPower(-power);
        robot.rightFront.setPower(-power);
        robot.leftBack.setPower(-power);
        runtime.reset();

        //loop until the range is reached or runtime runs out
        while (opModeIsActive() &&
                //The range sensor on the robot is not straight; therefore, using trigonometry
                //the value has to be adjusted
                (robot.sensorFront.getDistance(DistanceUnit.MM) > sensorMM) &&
                runtime.milliseconds() < timeoutMili) {

        }
        //stop robot
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.rightBack.setPower(0);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void sideDrive(double power, double x, String side, double timeoutMili) {

        int targetLF = robot.leftFront.getCurrentPosition() + (int)(x * countsPerCm);
        int targetRF = robot.rightFront.getCurrentPosition() + (int)(x * countsPerCm);
        int targetLB = robot.leftBack.getCurrentPosition() + (int)(x * countsPerCm);
        int targetRB = robot.rightBack.getCurrentPosition() + (int)(x * countsPerCm);

        double iPower = 0;
        double kPower = 0;
        if(side == "L") {
            iPower = power;
            kPower = -power;
            targetRF = -targetRF;
            targetLB = -targetLB;
        } else if(side == "R") {
            iPower = -power;
            kPower = power;
            targetLF = -targetLF;
            targetRB = -targetRB;
        }

        robot.leftFront.setTargetPosition(targetLF);
        robot.rightFront.setTargetPosition(targetRF);
        robot.leftBack.setTargetPosition(targetLB);
        robot.rightBack.setTargetPosition(targetRB);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();

        robot.leftFront.setPower(iPower);
        robot.rightBack.setPower(iPower);

        robot.rightFront.setPower(kPower);
        robot.leftBack.setPower(kPower);

        while (opModeIsActive() &&
                (runtime.milliseconds() < timeoutMili) &&
                (robot.leftFront.isBusy() && robot.rightFront.isBusy() && robot.leftBack.isBusy() && robot.rightFront.isBusy())) {

            // Display it for the driver.
                /*telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d",
                                            robot.leftFront.getCurrentPosition(),
                                            robot.rightFront.getCurrentPosition(),
                                            robot.leftBack.getCurrentPosition(),
                                            robot.rightBack.getCurrentPosition());*/
            //telemetry.update();
        }
        // Stop all motion;
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.rightBack.setPower(0);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void encoderDrive(double speed,
                             double x,  String way,
                             double timeoutS) {

        if (opModeIsActive()) {

            int targetLF = robot.leftFront.getCurrentPosition() + (int)(x * countsPerCm);
            int targetRF = robot.rightFront.getCurrentPosition() + (int)(x * countsPerCm);
            int targetLB = robot.leftBack.getCurrentPosition() + (int)(x * countsPerCm);
            int targetRB = robot.rightBack.getCurrentPosition() + (int)(x * countsPerCm);

            double iPower = 0;
            double kPower = 0;
            if(way == "F") {
                iPower = speed;
                kPower = speed;
            } else if(way == "L") {
                iPower = speed;
                kPower = -speed;
                targetRF = -targetRF;
                targetLB = -targetLB;
            } else if(way == "R") {
                iPower = -speed;
                kPower = speed;
                targetLF = -targetLF;
                targetRB = -targetRB;
            }

            robot.leftFront.setTargetPosition(targetLF);
            robot.rightFront.setTargetPosition(targetRF);
            robot.leftBack.setTargetPosition(targetLB);
            robot.rightBack.setTargetPosition(targetRB);

            // Turn On RUN_TO_POSITION
            robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            robot.leftFront.setPower(iPower);
            robot.rightBack.setPower(iPower);

            robot.rightFront.setPower(kPower);
            robot.leftBack.setPower(kPower);

            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (robot.leftFront.isBusy() && robot.rightFront.isBusy() && robot.leftBack.isBusy() && robot.rightFront.isBusy()) &&
                    robot.sensorRange.getDistance(DistanceUnit.MM)*Math.cos(0.57) > 60) {

                //telemetry.addData("range", String.format("%.01f mm", robot.sensorRange.getDistance(DistanceUnit.MM)));

                // Display it for the driver.
                /*telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d",
                                            robot.leftFront.getCurrentPosition(),
                                            robot.rightFront.getCurrentPosition(),
                                            robot.leftBack.getCurrentPosition(),
                                            robot.rightBack.getCurrentPosition());*/
                //telemetry.update();
            }
            // Stop all motion;
            robot.rightFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.leftFront.setPower(0);
            robot.rightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void resetEncoders() {
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private int analyze() {
        int[] result = measure();

        int smallestPos = 0;
        int x = 0;
        for (int i=0; i < 3; i++) {
            int ri = result[i];
            if(i==0) {
                x = ri;
            } else if(ri < x) {
                x = ri;
                smallestPos = i;
            }
        }
        return smallestPos;
    }

    private int[] measure() {
        int yellowLeft = 0;
        int yellowMiddle = 0;
        int yellowRight = 0;

        if (imageReady()) { // only do this if an image has been returned from the camera

            // get image, rotated so (0,0) is in the bottom left of the preview window
            Bitmap rgbImage;
            rgbImage = convertYuvImageToRgb(yuvImage, width, height, ds2);

            int width = rgbImage.getWidth();
            int height = rgbImage.getHeight();

            for (int x = 0; x < width; x++) {
                for (int y = 0; y < height; y++) {

                    if(y < height-30 && y > 40) {
                        int pixel = rgbImage.getPixel(x, y);
                        if (x <= width/3) {
                            yellowLeft += red(pixel) + green(pixel);
                        } else if(x > width/3 && x <= (width - width/3) ) {
                            yellowMiddle += red(pixel) + green(pixel);
                        } else {
                            yellowRight += red(pixel) + green(pixel);
                        }
                    }

                }
            }
        }
        int[] measurement = {yellowLeft, yellowMiddle, yellowRight};
        return measurement;

    }


}
