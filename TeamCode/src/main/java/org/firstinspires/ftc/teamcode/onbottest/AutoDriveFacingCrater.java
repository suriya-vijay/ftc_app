package org.firstinspires.ftc.teamcode.onbottest;




import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name="AutoDriveFacingCrater", group="Test")
//@Disabled
public class AutoDriveFacingCrater extends AutoDriveFacingDepot {



    public void runTasks(){

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        /**/
        runForward(-20, 5);

        turnDegrees(100, 4.0); // turn left
        runForward(-48, 6);
        turnDegrees(103, 4.0); // turn left
        runForward(-27, 5); // run forward into depot

        //turnDegrees(-90, 4.0); // turn right
        //runForward(-27, 5); // run forward into depot
        dropMarker();
        resetMarker();

    }
}
