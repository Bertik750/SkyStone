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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Drive OpMode", group="Linear Opmode")
public class BasicDriveMode extends LinearOpMode {

    // Declare OpMode members.

    HardwareRobot robot = new HardwareRobot();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.init(hardwareMap);

        robot.backLeft.setPosition(0.5);
        robot.backRight.setPosition(0.5);
        robot.picker.setPosition(0.2);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        double dipperPos = 0.5;
        boolean intakeOn = false;
        int baseTarget = 0;
        boolean baseRunToPosFlag = false;
        boolean baseInTakeFlag = true;





        while (opModeIsActive()) {

            double turnCoe = 0.5 * Range.clip(1 - gamepad1.left_trigger, 0.3, 1);
            double speedCoe = Range.clip(1 - gamepad1.left_trigger, 0.3, 1);

            if(gamepad1.dpad_down) {
                intakeOn = !intakeOn;
                if(intakeOn) {
                    robot.rightIntake.setPower(0.8);
                    robot.leftIntake.setPower(0.8);
                } else {
                    robot.rightIntake.setPower(0);
                    robot.leftIntake.setPower(0);
                }
            }
            if(gamepad1.dpad_up) {
                robot.rightIntake.setPower(-0.5);
                robot.leftIntake.setPower(-0.5);
            }


            if(gamepad1.dpad_right) {
                robot.autoArmDown.setPosition(0.75);
                robot.autoArmUp.setPosition(0.05);
            }

            double intakeLoadPow = 0;
            if(gamepad1.y) {
                intakeLoadPow = 1;
            }
            if(gamepad1.a) {
                intakeLoadPow = -1;
            }
            robot.pick2.setPower(-intakeLoadPow);
            robot.pick1.setPower(intakeLoadPow);

            if(gamepad1.b) {
                robot.pick2.setPower(-1);
            }
            if(gamepad1.x) {
                robot.pick2.setPower(1);
            }

            if(gamepad1.left_stick_button) {
                robot.backLeft.setPosition(0);
                robot.backRight.setPosition(1);
            }
            if(gamepad1.right_stick_button) {
                robot.backLeft.setPosition(0.5);
                robot.backRight.setPosition(0.5);
            }

            //Mecanum wheels drive algorithm
            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x * turnCoe;
            double iPower = Range.clip((y - x) * speedCoe, -1.0, 1.0);
            double kPower = Range.clip((y + x) * speedCoe, -1.0, 1.0);
            // Send calculated power to wheels
            //i
            robot.leftFront.setPower(Range.clip(iPower - turn, -1.0, 1.0));
            robot.rightBack.setPower(Range.clip(iPower + turn, -1.0, 1.0));
            //k
            robot.rightFront.setPower(Range.clip(kPower + turn, -1.0, 1.0));
            robot.leftBack.setPower(Range.clip(kPower - turn, -1.0, 1.0));

            //DRIVER TWO
            double liftPow = 0;
            if(gamepad2.right_trigger != 0) {
                liftPow = -gamepad2.right_trigger;
            } else if(gamepad2.left_trigger != 0) {
                liftPow = gamepad2.left_trigger;
            }
            robot.lift.setPower(liftPow);

            //Dipper control
            dipperPos = robot.dipper.getPosition();
            if(gamepad2.left_stick_button) {
                dipperPos = 0.6;
            }

            if(gamepad2.left_stick_y < 0) {
                dipperPos -= 0.003;
            }
            if(gamepad2.left_stick_y > 0) {
                dipperPos += 0.003;
            }
            if(gamepad2.right_stick_y > 0 && baseInTakeFlag) {
                //dipperPos = gamepad2.right_stick_y;
                dipperPos = 0.7;
            }
            if(gamepad2.right_stick_y < 0) {
                dipperPos = 0.35;
            }

            dipperPos = Range.clip(dipperPos, 0.35,0.7);
            if (dipperPos != robot.dipper.getPosition()) {
                robot.dipper.setPosition(dipperPos);
            }


            //Picker control
            if(gamepad2.a) {
                robot.picker.setPosition(0.7);
            }
            if(gamepad2.y) {
                robot.picker.setPosition(0.2);
            }

            //Base turn manual
            double adjustTurnPow = 0.3;
            if(!robot.baseTurn.isBusy()) {
                if(gamepad2.dpad_right) {
                    robot.baseTurn.setPower(adjustTurnPow);
                } else if(gamepad2.dpad_left) {
                    robot.baseTurn.setPower(-adjustTurnPow);
                } else {
                    robot.baseTurn.setPower(0);
                }
            }

            //Base turn automation
            if(gamepad2.b) {
                runtime.reset();
                robot.baseTurn.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                baseTarget = 875;
                robot.baseTurn.setTargetPosition(baseTarget);
                robot.baseTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                baseRunToPosFlag = true;
                baseInTakeFlag = false;
                robot.baseTurn.setPower(0.65);
            }

            if(gamepad2.x) {
                runtime.reset();
                robot.baseTurn.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                baseTarget = 0;
                robot.baseTurn.setTargetPosition(baseTarget);
                robot.baseTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                baseRunToPosFlag = true;
                baseInTakeFlag = true;
                robot.baseTurn.setPower(-0.8);
                robot.picker.setPosition(0.2);
            }
//            int distanceToTarget = robot.baseTurn.getTargetPosition() - robot.baseTurn.getCurrentPosition();

            /*
            if(baseRunToPosFlag && robot.baseTurn.isBusy() && abs || runtime.seconds() > 3 || gamepad2.dpad_right || gamepad2.dpad_left )) {

             */
            if(baseRunToPosFlag && (!robot.baseTurn.isBusy() || runtime.seconds() > 3 || gamepad2.dpad_right || gamepad2.dpad_left )) {
                baseRunToPosFlag = false;
                robot.baseTurn.setPower(0);
                robot.baseTurn.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if(baseInTakeFlag) {
                    robot.dipper.setPosition(0.75);
                }

            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "i (%.2f), k (%.2f)", iPower, kPower);
            telemetry.update();


        }
    }

    private void extend(double secs) {
        runtime.reset();
        while (runtime.milliseconds() < secs*1000) {
            robot.pick2.setPower(-1);
            robot.pick1.setPower(1);
        }
        robot.pick2.setPower(0);
        robot.pick1.setPower(0);
    }
    private void retract(double secs) {
        runtime.reset();
        while (runtime.milliseconds() < secs*1000) {
            robot.pick2.setPower(1);
            robot.pick1.setPower(-1);
        }
        robot.pick2.setPower(0);
        robot.pick1.setPower(0);
    }


}
