package org.firstinspires.ftc.teamcode.Current_Code.Auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Current_Code.Auto.Red;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
(name="V3R_Long", group="Robot")
public class V3R_Long extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor motorFL = null;
    private DcMotor motorFR = null;
    private DcMotor motorBL = null;
    private DcMotor motorBR = null;
    private DcMotor motorLSL = null;
    private DcMotor motorLSR = null;

    Servo servoArm;
    Servo servoIn;
    Servo servoBucket;
    Servo servoDropper;
    OpenCvWebcam webcam;

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
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorLSL = hardwareMap.dcMotor.get("motorLSL");
        motorLSR = hardwareMap.dcMotor.get("motorLSR");
        servoIn = hardwareMap.servo.get("servoIn");
        servoArm = hardwareMap.servo.get("servoArm");
        servoBucket = hardwareMap.servo.get("servoBucket");
        servoDropper = hardwareMap.servo.get("servoDropper");




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
                Move(directions.FORWARDS, 26, .15);
                Move(directions.COUNTERCLOCKWISE,19,.15);
                Move(directions.FORWARDS,5,.15);

                //drop
                servoDropper.setPosition(0.57);
                sleep(1000);

                //got to backdrop
                // Move(V3B_Short.directions.RIGHT, 12, .25);
                // Move(directions.COUNTERCLOCKWISE, 17, .25);
                //  Move(V3B_Short.directions.FORWARDS, 36, .25);
                //  Move(V3B_Short.directions.LEFT, 3, .25);


                //play on backdrop
                //   Move(directions.CLOCKWISE, 34, .25);
                //   slides(2, 1);
                //   sleep(500);
                //  servoArm.setPosition(1);
                //  sleep(1000);
                // servoBucket.setPosition(0);

                //park
                //park
                Move(directions.BACKWARDS,72,.25);
                Move(directions.LEFT,36, .5);
                Move(directions.BACKWARDS,48,.5);


                break;

            }


            case RIGHT: // Level 1
            {

                //go to target
                Move(directions.FORWARDS, 26, .15);
                Move(directions.CLOCKWISE,19,.15);
                Move(directions.FORWARDS,5,.15);
                Move(directions.LEFT,2,.15);

                //drop
                servoDropper.setPosition(.57);
                sleep(1000);

                //go to backdrop
                //  Move(directions.FORWARDS, 24, .25);
                // Move(directions.RIGHT, 3, .25);


                //play on backdrop
                // Move(directions.CLOCKWISE, 34, .25);
                // slides(2, 1);
                //  sleep(500);
                //servoArm.setPosition(1);
                // sleep(1000);
                //servoBucket.setPosition(0);

                //park
                Move(directions.BACKWARDS,24,.5);
                Move(directions.LEFT,26,.5);
                Move(directions.FORWARDS,109,.5);
                Move(directions.RIGHT,9,.25);
                Move(directions.FORWARDS,10,.25);

                break;
            }

            case CENTER: // Level 2
            {

                //go to target
                Move(directions.FORWARDS, 30, .15);

                //drop
                servoDropper.setPosition(0.57);
                sleep(1000);

                //go to backdrop
              //  Move(directions.RIGHT,3,.25);
              //  Move(directions.FORWARDS, 72, .25);

                //play on backdrop
              //  Move(directions.CLOCKWISE, 34, .25);
                //slides(2, 1);
            //    sleep(500);
              //  servoArm.setPosition(1);
                //sleep(1000);
               // servoBucket.setPosition(0);

                //park
                Move(directions.LEFT,24,.5);
                Move(directions.FORWARDS,28,.5);
                Move(directions.CLOCKWISE,19,.5);
                Move(directions.FORWARDS,72,.5);
                Move(directions.RIGHT,24,.25);
                Move(directions.FORWARDS,48,.5);


                break;
            }
        }

    }

    private void slides(int target, double speed){

        motorLSL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLSR.setDirection(DcMotorSimple.Direction.FORWARD);


        motorLSL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLSR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLSL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLSR.setDirection(DcMotorSimple.Direction.FORWARD);

        motorLSL.setTargetPosition(target * in);
        motorLSR.setTargetPosition(target * in);

        motorLSL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLSR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLSL.setPower(speed);
        motorLSR.setPower(speed);

        while (opModeIsActive() && ((motorFL.isBusy() || motorFR.isBusy()))) {
        }

        motorLSL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLSR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void Move(directions direction, int target, double speed) {
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // This sets the direction for the motor for the wheels to drive forward
        if (direction == V3R_Long.directions.FORWARDS) {
            motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
            motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
            motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
            motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        // Sets the motor direction to move Backwards
        else if (direction == V3R_Long.directions.BACKWARDS) {
            motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
            motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
            motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
            motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        // Sets the motor direction to move to the Left ( Note * Port = Left)
        else if (direction == V3R_Long.directions.LEFT) {
            motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
            motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
            motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
            motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        // Sets the motor direction to move to the Right (Note * Starboard = Right)
        else if (direction == V3R_Long.directions.RIGHT) {
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



