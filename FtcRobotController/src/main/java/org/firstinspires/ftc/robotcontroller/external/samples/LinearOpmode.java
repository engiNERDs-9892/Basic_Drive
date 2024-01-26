/* Copyright (c) 2021 FIRST. All rights reserved.
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

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
public class LinearOpmode extends LinearOpMode {

    private DcMotor motorLS;
    private DcMotor motorFL;
    private DcMotor motorFR;
    private DcMotor motorBL;
    private DcMotor motorBR;
    private DcMotor motorHL;
    private DcMotor motorHR;

    Servo servoRadial;
    Servo servoCL;
    Servo servoCR;
    Servo servoWR;
    Servo servoWL;
    Servo servoDrone;


    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        motorFL  = hardwareMap.get(DcMotor.class, "motorFL");
        motorBL  = hardwareMap.get(DcMotor.class, "motorBL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        motorHL = hardwareMap.get(DcMotor.class, "motorHL");
        motorHR = hardwareMap.get(DcMotor.class, "motorHR");
        servoCL = hardwareMap.servo.get("servoCL");
        servoCR = hardwareMap.servo.get("servoCR");
        servoRadial = hardwareMap.servo.get("servoRadial");
        servoWR = hardwareMap.servo.get("servoWR");
        servoWL = hardwareMap.servo.get("servoWL");
        servoDrone = hardwareMap.servo.get("servoDrone");

        motorLS.setPower(0);
        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);
        servoDrone.setPosition(.17);


        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorLS.setDirection(DcMotor.Direction.REVERSE);
        servoCR.setDirection(Servo.Direction.REVERSE);
        servoRadial.setDirection(Servo.Direction.REVERSE);
        servoWR.setDirection(Servo.Direction.REVERSE);
        servoWL.setDirection(Servo.Direction.FORWARD);
        servoDrone.setDirection(Servo.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;
            double ls = gamepad2.left_stick_y; // Slide up or down


            // CLAW//

//close claw
            if (gamepad2.a){
                servoCL.setPosition(0);
                servoCR.setPosition(0);
            }
//open claw
            if(gamepad2.b){
                servoCL.setPosition(.2);
                servoCR.setPosition(.2);
            }

            //ARM//

//arm up
            if(gamepad2.y) {
                servoRadial.setPosition(.66);

            }
//arm down

            if ( gamepad2.x){
                servoRadial.setPosition(.02);

            }

            //WRIST//

//wrist down
            if (gamepad2.left_trigger != 0) {
                servoWR.setPosition(.4);
                servoWL.setPosition(.4);

            }
//wrist up
            else if (gamepad2.right_trigger !=0) {
                servoWR.setPosition(.6);
                servoWL.setPosition(.6);
//wrist
            } else {
                servoWR.setPosition(.5);
                servoWL.setPosition(.5);
            }

//hook
            if(gamepad2.right_stick_y>0){
                motorHL.setPower(1);
                motorHR.setPower(1);
            }else if(gamepad2.right_stick_y<0){
                motorHL.setPower(-1);
                motorHR.setPower(-1);
            }else{
                motorHL.setPower(0);
                motorHR.setPower(0);
            }


            // LINEAR SLIDE//

            if (ls == 0) {
                motorLS.setPower(0);
            } else {
                // move slide up for ls < 0, move slide down on ls > 0
                motorLS.setPower(ls * 1);
            }

            //launch drone
            if(gamepad1.y){
                servoDrone.setPosition(.3);
            }


            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }


            // Send calculated power to wheels
            motorFL.setPower(leftFrontPower);
            motorFR.setPower(rightFrontPower);
            motorBL.setPower(leftBackPower);
            motorBR.setPower(rightBackPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
        }
    }}
