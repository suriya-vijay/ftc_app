package org.firstinspires.ftc.teamcode.onbottest;




import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="AutoDriveFacingDepot", group="Test")
//@Disabled
public class AutoDriveFacingDepot extends LinearOpMode {

    /* Declare OpMode members. */
    private MyBot robot = new MyBot();

    private ElapsedTime     runtime = new ElapsedTime();
    static final double     WHEEL_BASE_INCHES       = 12;
    static final double     COUNTS_PER_MOTOR_REV    = 288 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 4.0;

    static final double     TURN_SPEED              = 0.6;

    class MyBot
    {
        /* Public OpMode members. */
        public DcMotor leftDrive   = null;
        public DcMotor  rightDrive  = null;
        public DcMotor liftDrive = null;
        public Servo markerServo = null;
        public Servo hookServo = null;

        /* local OpMode members. */
        HardwareMap hwMap           =  null;
        private ElapsedTime period  = new ElapsedTime();

        /* Constructor */
        public MyBot(){

        }

        /* Initialize standard Hardware interfaces */
        public void init(HardwareMap ahwMap) {
            // Save reference to Hardware map
            hwMap = ahwMap;

            // Define and Initialize Motors
            leftDrive  = hwMap.get(DcMotor.class, "left_drive");
            rightDrive = hwMap.get(DcMotor.class, "right_drive");
            liftDrive = hwMap.get(DcMotor.class, "lift_drive");

            markerServo = hwMap.get(Servo.class, "marker_servo");
            hookServo = hwMap.get(Servo.class, "hook_servo");  //Hook server motor

            leftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
            rightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
            liftDrive.setDirection(DcMotor.Direction.FORWARD);

            // Set all motors to zero power
            leftDrive.setPower(0);
            rightDrive.setPower(0);
            liftDrive.setPower(0);



        }
    }

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

/*
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d - %7f" ,
                robot.leftDrive.getCurrentPosition(),
                robot.rightDrive.getCurrentPosition(),
                robot.markerServo.getPosition());
        telemetry.update();
*/
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // Do the actual work
        runTasks();

        //  sleep(1000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
    //runtime.seconds changed from 2.0
    public void dropMarker(){
        runtime.reset();
        while (opModeIsActive() &&
                (runtime.seconds() < 1.5)){
            robot.markerServo.setPosition(0.5);

        }
    }
    public void hookRelease(){
        runtime.reset();
        while (opModeIsActive() &&
                (runtime.seconds() < 2.5)){
            robot.hookServo.setPosition(0.5);

        }
    }



    public void resetMarker(){
        runtime.reset();

        while (opModeIsActive() &&
                (runtime.seconds() < 1.5)){
            robot.markerServo.setPosition(0);

        }
    }
    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftDrive.setPower(Math.abs(speed));
            robot.rightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftDrive.getCurrentPosition(),
                        robot.rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void liftDrive(double speed,
                          double liftInches,
                          double timeoutS) {
        int newLiftTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLiftTarget = robot.liftDrive.getCurrentPosition() + (int)(liftInches * (COUNTS_PER_MOTOR_REV/(0.5 *3.14)) );
            robot.liftDrive.setTargetPosition(newLiftTarget);

            // Turn On RUN_TO_POSITION
            robot.liftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.liftDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.liftDrive.isBusy() )) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Lifting to %7d ", newLiftTarget);
                telemetry.addData("Path2",  "Lifting from %7d ",
                        robot.liftDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.liftDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.liftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }


    protected void runForward(double inches, double timeoutS ){
        encoderDrive(DRIVE_SPEED,  inches,  inches, timeoutS);
    }

    protected void lift(double inches, double timeoutS ){
        liftDrive(0.5,  inches,  timeoutS);
    }

    protected void turnDegrees(double angle, double timeoutS){
        double archLength = 3.1415 * WHEEL_BASE_INCHES * angle /360;
        encoderDrive(TURN_SPEED,   archLength, -1*archLength, 4.0);  // turn left

    }


    public void runTasks(){

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        /**/
        // lift(20.0,2);
        runForward(-4, 5); // first move forward a little
        turnDegrees(45, 6.0); // turn left
        runForward(-44, 5); // run forward past the mineral marker
        turnDegrees(-101, 6.0); // turn towards the depot
        runForward(-36, 5); // run into depot
        dropMarker();
        resetMarker();
        runForward(110, 5); // run towards the crater

    }
}
