package org.firstinspires.ftc.teamcode.Current_Code.DriverControl_Code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.Servo;



@TeleOp

public class Driver_Red extends LinearOpMode {
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
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorLS = hardwareMap.dcMotor.get("motorLS");
        motorHL = hardwareMap.dcMotor.get("motorHL");
        motorHR = hardwareMap.dcMotor.get("motorHR");
        servoCL = hardwareMap.servo.get("servoCL");
        servoCR = hardwareMap.servo.get("servoCR");
        servoRadial = hardwareMap.servo.get("servoRadial");
        servoWR = hardwareMap.servo.get("servoWR");
        servoWL = hardwareMap.servo.get("servoWL");

        motorLS.setPower(0);
        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);
        motorHL.setPower(0);
        motorHR.setPower(0);

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.

        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorLS.setDirection(DcMotor.Direction.REVERSE);
        motorHL.setDirection(DcMotor.Direction.FORWARD);
        motorHR.setDirection(DcMotor.Direction.FORWARD);
        servoCR.setDirection(Servo.Direction.REVERSE);
        servoRadial.setDirection(Servo.Direction.REVERSE);
        servoWR.setDirection(Servo.Direction.REVERSE);
        servoWL.setDirection(Servo.Direction.FORWARD);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "IMU");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (isStopRequested()) return;


        while (opModeIsActive()) {
            int motorLSpos = motorLS.getCurrentPosition();
            double ls = gamepad2.left_stick_y; // Slide up or down
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident
            // it can be freely changed based on preference.

            if (gamepad1.dpad_down) {
                // sets facing direction 'forward'
                imu.resetYaw();
            }

            //CLAW//

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

            // LINEAR SLIDE//

            if (ls == 0) {
                motorLS.setPower(0);
            } else {
                // move slide up for ls < 0, move slide down on ls > 0
                motorLS.setPower(ls * 1);
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

            //drone
            if(gamepad1.a){
                servoDrone.setPosition(.4);
            }

            // wheels//


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

            if (gamepad1.left_trigger  !=0) {
                motorFL.setPower(.3 * frontLeftPower);
                motorBL.setPower(.3 * backLeftPower);
                motorFR.setPower(.3 * frontRightPower);
                motorBR.setPower(.3 * backRightPower);
            } else {
                motorFL.setPower(.65 * frontLeftPower);
                motorBL.setPower(.65 * backLeftPower);
                motorFR.setPower(.65 * frontRightPower);
                motorBR.setPower(.65 * backRightPower);
            }
        }

    }
}

