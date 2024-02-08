package org.firstinspires.ftc.teamcode.Current_Code.Auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="R_Short", group="Robot")
public class R_Short extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor motorFL = null;
    private DcMotor motorFR = null;
    private DcMotor motorBL = null;
    private DcMotor motorBR = null;

    Servo servoArm;
    Servo servoC;    OpenCvWebcam webcam;
    Red.SkystoneDeterminationPipeline pipeline;
    Red.SkystoneDeterminationPipeline.SkystonePosition snapshotAnalysis = Red.SkystoneDeterminationPipeline.SkystonePosition.RIGHT;
    int in = 45;

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()


    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double COUNTS_PER_MOTOR_REV = 1120;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);



    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        pipeline = new Red.SkystoneDeterminationPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // This is in what viewing window the camera is seeing through and it doesn't matter
                // what orientation it is | UPRIGHT, SIDEWAYS_LEFT, SIDEWAYS_RIGHT, etc.
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Realtime analysis", pipeline.getAnalysis());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }

        snapshotAnalysis = pipeline.getAnalysis();


        telemetry.addData("Snapshot post-START analysis", snapshotAnalysis);
        telemetry.update();

        // Initialize the drive system variables.
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        servoC = hardwareMap.servo.get("servoC");
        servoArm =hardwareMap.servo.get("servoArm");




        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips



        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        telemetry.addData("Status", "\uD83C\uDD97");
        telemetry.update();


        waitForStart();



        switch (snapshotAnalysis) {
            case LEFT: // Level 3
            {

                //go to target
                Move(directions.FORWARDS, 24, .25);
                Move(directions.COUNTERCLOCKWISE, 17, .25);


                //drop pixel
                servoArm.setPosition(1);
                servoC.setPosition(0);
                sleep(1000);
                servoArm.setPosition(0.09);
                sleep(500);
                servoC.setPosition(0);

                //play on backboard
                Move(directions.CLOCKWISE, 17, .25);
                Move(directions.FORWARDS, 16, .25);
                Move(directions.CLOCKWISE, 17, .25);
                servoArm.setPosition(.7);
                Move(directions.FORWARDS, 12,.25);
                servoC.setPosition(0);

                //park
                Move(directions.BACKWARDS, 12, .25);
                Move(directions.RIGHT, 48, .25);
                Move(directions.FORWARDS, 24, .25);

                break;

            }


            case RIGHT: // Level 1
            {
                servoC.setPosition(0);
                sleep(500);
                servoArm.setPosition(0.3);

                //go to target
                Move(directions.FORWARDS, 24, .25);
                Move(directions.LEFT, 3, .25);
                Move(directions.CLOCKWISE, 16, .25);
                Move(directions.FORWARDS, 9, .25);
                Move(directions.BACKWARDS, 9, .25);

                //drop pixel
                servoArm.setPosition(0.95);
                servoC.setPosition(0.09);
                sleep(1000);
                servoArm.setPosition(0);
                sleep(1000);
                servoC.setPosition(.09);

                //play on backdrop
               Move(directions.CLOCKWISE, 16, .25);
               Move(directions.FORWARDS, 12, .25);
               Move(directions.COUNTERCLOCKWISE, 17, .25);
               Move(directions.FORWARDS, 12, .25);
               servoC.setPosition(0.7);
               servoC.setPosition(0);

               //park
                servoArm.setPosition(0.1);
                Move(directions.RIGHT, 12, .25);
                Move(directions.FORWARDS, 24, .25);

                break;
            }

            case CENTER: // Level 2
            {

                //go to target
                Move(directions.FORWARDS, 30, .25);
                Move(directions.BACKWARDS, 4, .25);

                //drop the pixel and pick up yellow
                servoArm.setPosition(1);
                servoC.setPosition(0);
                sleep(1000);
                servoArm.setPosition(0.09);
                sleep(500);
                servoC.setPosition(0);

                //play on backboard
                Move(directions.CLOCKWISE, 17, .25);
                servoArm.setPosition(0.7);
                Move(directions.FORWARDS, 15, .25);
                servoC.setPosition(0);

                //park
                Move(directions.BACKWARDS, 4, .25);
                Move(directions.RIGHT, 24, .25);
                Move(directions.FORWARDS, 24, .25);

                break;
            }
        }

    }


    private void Move(directions direction, int target, double speed) {
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // This sets the direction for the motor for the wheels to drive forward
        if (direction == R_Short.directions.FORWARDS) {
            motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
            motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
            motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
            motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        // Sets the motor direction to move Backwards
        else if (direction == R_Short.directions.BACKWARDS) {
            motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
            motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
            motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
            motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        // Sets the motor direction to move to the Left ( Note * Port = Left)
        else if (direction == R_Short.directions.LEFT) {
            motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
            motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
            motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
            motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        // Sets the motor direction to move to the Right (Note * Starboard = Right)
        else if (direction == R_Short.directions.RIGHT) {
            motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
            motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
            motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
            motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else if (direction == directions.CLOCKWISE) {
            motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
            motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
            motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
            motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else if (direction == directions.COUNTERCLOCKWISE) {
            motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
            motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
            motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
            motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        // Gives it a position to run to
        motorFL.setTargetPosition(target * in);
        motorFR.setTargetPosition(target * in);
        motorBL.setTargetPosition(target * in);
        motorBR.setTargetPosition(target * in);

        // tells it to go to the position that is set
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // the motor speed for Wheels
        motorFL.setPower(speed);
        motorFR.setPower(speed);
        motorBL.setPower(speed);
        motorBR.setPower(speed);



        // While loop keeps the code running until motors reach the desired position
        while (opModeIsActive() && ((motorFL.isBusy() || motorFR.isBusy()))) {
        }
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    enum directions {
        FORWARDS,
        BACKWARDS,
        LEFT,
        RIGHT,
        CLOCKWISE,
        COUNTERCLOCKWISE
    }}


