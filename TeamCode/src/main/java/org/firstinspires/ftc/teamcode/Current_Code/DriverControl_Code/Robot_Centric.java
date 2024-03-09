package org.firstinspires.ftc.teamcode.Current_Code.DriverControl_Code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Robot_Centric extends LinearOpMode {

    private DcMotor motorFL;
    private DcMotor motorFR;
    private DcMotor motorBL;
    private DcMotor motorBR;
    private DcMotor motorLSL;
    private DcMotor motorLSR;
    private DcMotor motorLS;
    Servo servoIn;
    Servo servoArm;
    Servo servoBucket;
    Servo servoPlane;
    Servo servoHangL;
    Servo servoHangR;
    


    @Override
    public void runOpMode() {
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorLSL = hardwareMap.dcMotor.get("motorLSL");
        motorLSR = hardwareMap.dcMotor.get("motorLSR");
        servoIn = hardwareMap.servo.get("servoIn");
        servoArm = hardwareMap.servo.get("servoArm");
        servoBucket = hardwareMap.servo.get("servoBucket");
        servoPlane = hardwareMap.servo.get("servoPlane");
        motorLS = hardwareMap.dcMotor.get("motorLS");
        servoHangL = hardwareMap.servo.get("servoHangL");
        servoHangR = hardwareMap.servo.get("servoHangR");



        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);
        motorLSL.setPower(0);
        motorLSR.setPower(0);


        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorLSL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLS.setDirection(DcMotorSimple.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();


        // run until the end of the match (driver presses STOP) uwu
        while (opModeIsActive()) {
            double max;
            double ls = gamepad2.left_stick_y; //intake uwu
            double ls2 = gamepad2.right_stick_y; //Slide up or down uwu

            //turn  on intake system uwu
            if (gamepad2.right_bumper) {
                servoIn.setPosition(-0.6);
            }

            if (gamepad2.left_trigger != 0) {
                servoIn.setPosition(0.5);
            }

            if (gamepad2.right_trigger != 0) {
                servoIn.setPosition(0.5);
            }

            if (gamepad2.left_bumper) {
                servoIn.setPosition(0.6);
            }

            //Plane Launcher
            if (gamepad1.dpad_right) {
                servoPlane.setPosition(0.25);
            }

            if (gamepad1.dpad_left) {
                servoPlane.setPosition(0);
            }



            //Arrrm uup /dowwn

            if (gamepad2.x) {
                servoArm.setPosition(.16);
            }
            if (gamepad2.y) {
                servoArm.setPosition(.7);
            }

                        //claw

            if (gamepad2.b) {
                servoBucket.setPosition(1);
            }
            else if (gamepad2.a) {
                servoBucket.setPosition(0);
            } else {
                servoBucket.setPosition(0.5);
            }


                        //LINEAR SLIDE UWU
                        if (ls == 0) {
                            motorLSL.setPower(0);
                            motorLSR.setPower(0);
                        } else {
                            // move slide up for ls < 0, move slide down on ls > 0
                            motorLSL.setPower(ls * 0.9);
                            motorLSR.setPower(ls * 0.9);
                        }


            if (ls2 == 0) {
                motorLS.setPower(0);
            } else {
                // move slide up for ls < 0, move slide down on ls > 0
                motorLS.setPower(ls2 * 1);
            }

            //hanging
            if (gamepad1.right_bumper){
                servoHangL.setPosition(0);
                servoHangR.setPosition(0);
            } else if (gamepad1.left_bumper) {
                servoHangR.setPosition(1);
                servoHangL.setPosition(1);

            }else{
                servoHangL.setPosition(0.5);
                servoHangL.setPosition(0.5);
            }


            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
                        double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
                        double lateral = gamepad1.left_stick_x;
                        double yaw = gamepad1.right_stick_x;

                        // Combine the joystick requests for each axis-motion to determine each wheel's power.
                        // Set up a variable for each drive wheel to save the power level for telemetry.
                        double leftFrontPower = axial + lateral + yaw;
                        double rightFrontPower = axial - lateral - yaw;
                        double leftBackPower = axial - lateral + yaw;
                        double rightBackPower = axial + lateral - yaw;

                        // Normalize the values so no wheel power exceeds 100%
                        // This ensures that the robot maintains the desired motion.
                        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
                        max = Math.max(max, Math.abs(leftBackPower));
                        max = Math.max(max, Math.abs(rightBackPower));

                        if (max > 1.0) {
                            leftFrontPower /= max;
                            rightFrontPower /= max;
                            leftBackPower /= max;
                            rightBackPower /= max;
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

                        //speeds
                        if (gamepad1.left_trigger != 0) {
                            motorFL.setPower(.25 * leftFrontPower);
                            motorBL.setPower(.25 * leftBackPower);
                            motorFR.setPower(.25 * rightFrontPower);
                            motorBR.setPower(.25 * rightBackPower);

                        }    else if (gamepad1.right_trigger != 0) {
                                motorFL.setPower(.9 * leftFrontPower);
                                motorBL.setPower(.9 * leftBackPower);
                                motorFR.setPower(.9 * rightFrontPower);
                                motorBR.setPower(.9 * rightBackPower);
                            }
                         else {
                            motorFL.setPower(.65 * leftFrontPower);
                            motorBL.setPower(.65 * leftBackPower);
                            motorFR.setPower(.65 * rightFrontPower);
                            motorBR.setPower(.65 * rightBackPower);
                        }

                    }
                }
            }

