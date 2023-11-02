package org.firstinspires.ftc.teamcode.Current_Code.DriverControl_Code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.Servo;



@TeleOp
public class DriverPeriod extends LinearOpMode {
    private DcMotor motorLS;

    Servo servoRadial;
    Servo servoLadial;

    // The Servo ____; are used to identify a servo that can be used throughout the code.

    Servo servoCL;
    Servo servoCR;
    Servo servoWR;
    Servo servoWL;


    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor motorFL = hardwareMap.dcMotor.get("motorFL");
        DcMotor motorBL = hardwareMap.dcMotor.get("motorBL");
        DcMotor motorFR = hardwareMap.dcMotor.get("motorFR");
        DcMotor motorBR = hardwareMap.dcMotor.get("motorBR");
        motorLS = hardwareMap.dcMotor.get("motorLS");
        servoCL = hardwareMap.servo.get("servoCL");
        servoCR = hardwareMap.servo.get("servoCR");
        servoRadial = hardwareMap.servo.get("servoRadial");
        servoLadial = hardwareMap.servo.get("servoLadial");
        servoWR = hardwareMap.servo.get("servoWR");
        servoWL = hardwareMap.servo.get("servoWL");


        motorLS.setPower(0);
        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.

        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorLS.setDirection(DcMotor.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "IMU");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            int motorLSpos = motorLS.getCurrentPosition();
            double ls = gamepad2.left_stick_y; // Slide up or down
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            if (gamepad2.x) {
                servoRadial.setPosition(180);
                servoLadial.setPosition(0);

                sleep(500);

                servoWR.setPosition(180);
                servoWL.setPosition(0);
            }

            if (gamepad2.y) {
                servoRadial.setPosition(0);
                servoLadial.setPosition(180);

                sleep(500);

                servoWR.setPosition(0);
                servoWL.setPosition(180);
            }


            if (ls < 0) {
                motorLS.setPower(ls);

            }

            if (ls > 0 ) {
                motorLS.setPower(0);

            }

            if (ls > 0 ) {
                motorLS.setPower(ls * 1);

            }

            if (ls == 0) {
                motorLS.setPower(0);

            }
            if (motorLSpos < -11300 && ls<0 ) {
                motorLS.setPower(0);

            }


            if (motorLSpos > -150 && ls>0) {

                motorLS.setPower(0);

            }

            telemetry.addData("Position", motorLSpos);
            telemetry.update();

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

            motorFL.setPower(.6 * frontLeftPower);
            motorBL.setPower(.6 * backLeftPower);
            motorFR.setPower(.6 * frontRightPower);
            motorBR.setPower(.6 * backRightPower);
        }
    }
}
