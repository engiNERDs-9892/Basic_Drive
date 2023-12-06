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

@Autonomous(name="AutoRL", group="Robot")
public class AutoRL extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor motorFL = null;
    private DcMotor motorFR = null;
    private DcMotor motorBL = null;
    private DcMotor motorBR = null;
    private Servo servoRadial;

    private Servo servoWR;
    private Servo servoWL;
    private Servo servoCR;
    private Servo servoCL;

    private DcMotor motorLS = null;
    OpenCvWebcam webcamRed;
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
        webcamRed = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new Red.SkystoneDeterminationPipeline();
        webcamRed.setPipeline(pipeline);

        webcamRed.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // This is in what viewing window the camera is seeing through and it doesn't matter
                // what orientation it is | UPRIGHT, SIDEWAYS_LEFT, SIDEWAYS_RIGHT, etc.

                webcamRed.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
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
        motorFR = hardwareMap.get(DcMotor.class, "motorBL");
        motorFL = hardwareMap.get(DcMotor.class, "motorBR");
        motorBR = hardwareMap.get(DcMotor.class, "motorFL");
        motorBL = hardwareMap.get(DcMotor.class, "motorFR");
        motorLS = hardwareMap.get(DcMotor.class, "motorLS");
        servoRadial = hardwareMap.servo.get("servoRadial");
        servoWR = hardwareMap.servo.get("servoWR");
        servoWL = hardwareMap.servo.get("servoWL");
        servoCL = hardwareMap.servo.get("servoCL");
        servoCR = hardwareMap.servo.get("servoCL");



        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips



        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        telemetry.addData("Status", "\uD83C\uDD97");
        telemetry.update();

        waitForStart();

        switch (snapshotAnalysis) {
            case LEFT: // Level 3
            {

                //prep
                closeClaw();
                sleep(100);

                //go to target
                Move(directions.LEFT, 40, .25);
                sleep(500);
                Move(directions.BACKWARDS, 3, .25);

                //drop the pixel
                sleep(1000);
                openClaw();
                sleep(500);


                //park
                Move(directions.RIGHT, 6, .25);
                Move(directions.FORWARDS, 78, .25);
                Move(directions.RIGHT, 24, .25);
                Move(directions.FORWARDS, 12, .25);
                break;

            }


            case RIGHT: // Level 1
            {
                //prep
                closeClaw();
                sleep(500);

                //go to target
                Move(directions.LEFT, 29, .25);
                Move(directions.FORWARDS, 20, .25);
                Move(directions.LEFT, 6, .25);
                sleep(500);

                //drop the pixel
                openClaw();
                sleep(500);

                //park
                Move(directions.FORWARDS, 54, .25);
                Move(directions.RIGHT, 26, .25);
                Move(directions.FORWARDS, 6, .25);
                break;
            }

            case CENTER: // Level 2
            {

                //prep
                closeClaw();
                sleep(500);

                //go to target
                Move(directions.LEFT, 35, .25);
                Move(directions.COUNTERCLOCKWISE, 24, .25);
                Move(directions.BACKWARDS, 1, .25);

                //drop the pixel
                sleep(1000);
                openClaw();
                sleep(1000);


                //park
                Move(directions.FORWARDS, 5, .25);
                Move(directions.LEFT, 90, .25);
                Move(directions.BACKWARDS, 26, .25);
                Move(directions.LEFT, 20, .25);
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
        if (direction == directions.FORWARDS) {
            motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
            motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
            motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
            motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        // Sets the motor direction to move Backwards
        else if (direction == directions.BACKWARDS) {
            motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
            motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
            motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
            motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        // Sets the motor direction to move to the Left ( Note * Port = Left)
        else if (direction == directions.LEFT) {
            motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
            motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
            motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
            motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        // Sets the motor direction to move to the Right (Note * Starboard = Right)
        else if (direction == directions.RIGHT) {
            motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
            motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
            motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
            motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        else if (direction == directions.CLOCKWISE) {
            motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
            motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
            motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
            motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        else if (direction == directions.COUNTERCLOCKWISE) {
            motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
            motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
            motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
            motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
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
    }

    public void lsUp(double speed, int distance){
        int moveCounts = (int) (distance * COUNTS_PER_INCH);

        motorLS.setDirection(DcMotorSimple.Direction.REVERSE);


        motorLS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        motorLS.setTargetPosition(distance * in);


        motorLS.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        motorLS.setPower(-speed);

        sleep(500);






        while (opModeIsActive() && motorFL.isBusy()) {
        }

        motorLS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void lsDown(double speed, int distance){
        int moveCounts = (int) (distance * COUNTS_PER_INCH);

        motorLS.setDirection(DcMotorSimple.Direction.FORWARD);


        motorLS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        motorLS.setTargetPosition(distance * in);


        motorLS.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        motorLS.setPower(-speed);

        sleep(500);








        while (opModeIsActive() && motorFL.isBusy()) {
        }

        motorLS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }
    public void wristUp(){
        servoWR.setPosition(.6);
        servoWL.setPosition(.6);
    }
    public void wristDown() {
        servoWR.setPosition(.4);
        servoWL.setPosition(.4);
    }
    public void openClaw(){
        servoCR.setPosition(.3);
        servoCL.setPosition(.2);
    }
    public void closeClaw(){
        servoCR.setPosition(0);
        servoCL.setPosition(0);
    }
    public void Radial(){

        servoRadial.setPosition(.03);
    }

}