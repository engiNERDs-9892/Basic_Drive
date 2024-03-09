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
    private DcMotor motorLS;
    Servo servoHangL;
    Servo servoHangR;
    Servo servoIn;
    Servo servoArm;
    Servo servoBucket;
    Servo servoPlane;

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
        servoBucket = hardwareMap.servo.get("servoBucket");
        servoPlane = hardwareMap.servo.get("servoPlane");
        servoHangR = hardwareMap.servo.get("servoHangR");
        servoHangL = hardwareMap.servo.get("servoHangL");
        motorLS = hardwareMap.dcMotor.get("motorLS");

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
        motorLS.setDirection(DcMotorSimple.Direction.REVERSE);


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


        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double ls = gamepad2.left_stick_y; // intake
            double ls2 = gamepad2.right_stick_y; // up/down

            // This button choice was made so that it is hard to hit on accident
            //  it can be freely changed based on preference.
            if (gamepad1.dpad_down) {
                // sets facing direction 'forward'
                imu.resetYaw();
            }

            //turn on intake system
            if (gamepad2.right_bumper) {
                servoIn.setPosition(0.3);
            }

            if (gamepad2.left_trigger != 0) {
                servoIn.setPosition(0.5);
            }

            if (gamepad2.right_trigger != 0) {
                servoIn.setPosition(0.5);
            }

            if (gamepad2.left_bumper) {
                servoIn.setPosition(0.7);
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

            //claw thingy uwu
            if (gamepad2.b) {
                servoBucket.setPosition(1);
            }
          else if (gamepad2.a) {
                servoBucket.setPosition(0);
            } else {
              servoBucket.setPosition(0.5);
            }

            //Plane Launcher
            if (gamepad1.dpad_right) {
                servoPlane.setPosition(0.25);
            }
            if (gamepad1.dpad_left) {
                servoPlane.setPosition(0);
            }

            //Arm up & down uwu

            if (gamepad2.x) {
                servoArm.setPosition(.12);
            }
            if (gamepad2.y) {
                servoArm.setPosition(.8);
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
            if (gamepad1.left_trigger != 0) {
                motorFL.setPower(.3 * frontLeftPower);
                motorBL.setPower(.3 * backLeftPower);
                motorFR.setPower(.3 * frontRightPower);
                motorBR.setPower(.3 * backRightPower);

            }   else if (gamepad1.right_trigger != 0) {
                    motorFL.setPower(1 * frontLeftPower);
                    motorBL.setPower(1 * backLeftPower);
                    motorFR.setPower(1 * frontRightPower);
                    motorBR.setPower(1 * backRightPower);
                }
            else{
                motorFL.setPower(.8 * frontLeftPower);
                motorBL.setPower(.8 * backLeftPower);
                motorFR.setPower(.8 * frontRightPower);
                motorBR.setPower(.8 * backRightPower);

            }


            //LINEAR SLIDE
            if (ls == 0) {
                motorLSL.setPower(0);
                motorLSR.setPower(0);
            } else {
                // move slide up for ls < 0, move slide down on ls > 0
                motorLSL.setPower(ls * 1);
                motorLSR.setPower(ls * 1);
            }

            if (ls2 == 0) {
                motorLS.setPower(0);
            } else {
                // move slide up for ls < 0, move slide down on ls > 0
                motorLS.setPower(ls2 * 1);
            }

        }
    }
}