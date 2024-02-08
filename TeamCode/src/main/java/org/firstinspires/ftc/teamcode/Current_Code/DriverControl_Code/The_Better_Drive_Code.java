package org.firstinspires.ftc.teamcode.Current_Code.DriverControl_Code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class The_Better_Drive_Code extends LinearOpMode {

    private DcMotor motorFL;
    private DcMotor motorFR;
    private DcMotor motorBL;
    private DcMotor motorBR;
    private DcMotor motorLSL;
    private DcMotor motorLSR;
    Servo servoIn;
    Servo servoArm;
    Servo servoIn2;
    Servo servoC;
    Servo servoHangL;
    Servo servoHangR;
    Servo servoPlane;


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
        servoC = hardwareMap.servo.get("servoC");
        servoIn2 = hardwareMap.servo.get("servoIn2");
        servoHangL = hardwareMap.servo.get("servoHangL");
        servoHangR = hardwareMap.servo.get("servoHangR");
        servoPlane = hardwareMap.servo.get("servoPlane");

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

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;
            double ls = gamepad2.left_stick_y; //Slide up or down

            //turn  on intake system
            if (gamepad2.right_bumper) {
                servoIn.setPosition(1);
                servoIn2.setPosition(1);
            }

            if (gamepad2.right_trigger != 0) {
                servoIn.setPosition(0.5);
                servoIn2.setPosition(0.5);
            }

            if (gamepad2.left_trigger != 0) {
                servoIn.setPosition(0.5);
                servoIn2.setPosition(0.5);
            }

            if (gamepad2.left_bumper) {
                servoIn.setPosition(0.1);
                servoIn2.setPosition(0.1);
            }
            //Plane Launcher
            if (gamepad1.dpad_right) {
                servoPlane.setPosition(0.25);

                {
                    if (gamepad1.dpad_left) {
                        servoPlane.setPosition(0);

                        //Suspension
                        if (gamepad1.x) {
                            servoHangL.setPosition(0.66);
                            servoHangR.setDirection(Servo.Direction.REVERSE);
                            servoHangR.setPosition(0.66);
                        }
                        if (gamepad2.y) {
                            servoHangL.setPosition(0.33);
                            servoHangR.setDirection(Servo.Direction.REVERSE);
                            servoHangR.setPosition(0.33);
                        }

                        if (gamepad2.b) {
                            servoHangL.setPosition(0);
                            servoHangR.setDirection(Servo.Direction.REVERSE);
                            servoHangR.setPosition(0);
                        }


                        //Arrrm uup /dowwn
                        if (gamepad2.dpad_down) {
                            servoArm.setPosition(0);
                        }
                        if (gamepad2.x) {
                            servoArm.setPosition(.1);
                        }
                        if (gamepad2.y) {
                            servoArm.setPosition(.65);
                        }

                        //claw

                        if (gamepad2.b) {
                            servoC.setPosition(0.09);
                        }
                        if (gamepad2.a) {
                            servoC.setPosition(0);
                        }


                        //LINEAR SLIDE
                        if (ls == 0) {
                            motorLSL.setPower(0);
                            motorLSR.setPower(0);
                        } else {
                            // move slide up for ls < 0, move slide down on ls > 0
                            motorLSL.setPower(ls * 0.9);
                            motorLSR.setPower(ls * 0.9);
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
                        } else {
                            motorFL.setPower(.65 * leftFrontPower);
                            motorBL.setPower(.65 * leftBackPower);
                            motorFR.setPower(.65 * rightFrontPower);
                            motorBR.setPower(.65 * rightBackPower);
                        }
                        if (gamepad1.right_trigger != 0) {
                            motorFL.setPower(.9 * leftFrontPower);
                            motorBL.setPower(.9 * leftBackPower);
                            motorFR.setPower(.9 * rightFrontPower);
                            motorBR.setPower(.9 * rightBackPower);
                        }
                    }
                }
            }
        }
    }
}
