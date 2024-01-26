package org.firstinspires.ftc.teamcode.Current_Code.DriverControl_Code;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp

public class FieldCentric extends LinearOpMode {

    private DcMotor motorFL;
    private DcMotor motorFR;
    private DcMotor motorBL;
    private DcMotor motorBR;
    private DcMotor motorLSL;
    private DcMotor motorLSR;
    Servo servoIn;
    Servo servoArm;
    Servo servoC;
    @Override

    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorLSL = hardwareMap.dcMotor.get("motorLSL");
        motorLSR = hardwareMap.dcMotor.get("motorLSR");
        servoIn = hardwareMap.servo.get("servoIn");
        servoArm = hardwareMap.servo.get("servoArm");
        servoC = hardwareMap.servo.get("servoC");

        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);
        motorLSL.setPower(0);
        motorLSR.setPower(0);
        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.

        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorLSL.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "IMU");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (isStopRequested()) return;


        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double ls = gamepad2.left_stick_y; // Slide up or down
            // This button choice was made so that it is hard to hit on accident
            // it can be freely changed based on preference.

            if (gamepad1.dpad_down) {
                // sets facing direction 'forward'
                imu.resetYaw();
            }

            //turn  on intake system
            if (gamepad2.right_bumper) {
                servoIn.setPosition(1);
            }
            if (gamepad2.right_trigger !=0) {
                servoIn.setPosition(0.5);
            }

            //clawww thinngy
            if (gamepad2.b) {
                servoC.setPosition(0.07);
            }
            if (gamepad2.a) {
                servoC.setPosition(0);
            }

            //Arrrm  uup /dowwn
            if (gamepad2.x) {
                servoArm.setPosition(.02);
            }
            if (gamepad2.y) {
                servoArm.setPosition(.7);
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;


            //speeds
            if (gamepad1.left_trigger  !=0) {
                motorFL.setPower(.3 * frontLeftPower);
                motorBL.setPower(.3* backLeftPower);
                motorFR.setPower(.3 * frontRightPower);
                motorBR.setPower(.3 * backRightPower);
            } else {
                motorFL.setPower(.8 * frontLeftPower);
                motorBL.setPower(.8 * backLeftPower);
                motorFR.setPower(.8 * frontRightPower);
                motorBR.setPower(.8 * backRightPower);

            }
            if(gamepad1.right_trigger !=0){
                motorFL.setPower(1 * frontLeftPower);
                motorBL.setPower(1 * backLeftPower);
                motorFR.setPower(1 * frontRightPower);
                motorBR.setPower(1 * backRightPower);
            }

            //LINEAR SLIDE

            if (ls == 0) {
                motorLSL.setPower(0);
                motorLSR.setPower(0);
            } else {
                // move slide up for ls < 0, move slide down on ls > 0
                motorLSL.setPower(ls * 0.95);
                motorLSR.setPower(ls * 0.95);
            }
        }
    }

}



