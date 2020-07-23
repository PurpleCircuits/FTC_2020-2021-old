package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Most of this code was copied from the FTC examples, but we tweaked it for our purposes.
 *
 * This makes the robot drive forward and turn left and go forward, parking under the bridge during
 * autonomous gaining us 5 points.
 */
@Disabled
@Autonomous(name = "MoveFoundation", group = "Linear Opmode")
public class MoveFoundation extends LinearOpMode {

    // The speed for the drive motors to operate at during autonomous
    private static final double SPEED = 0.3;

    // Declare hardware
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor theSlide = null;

    // Used for determining how long something has ran
    private ElapsedTime runtime = new ElapsedTime();

    /**
     * This is the entry of our Op Mode.
     */
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "MoveFoundation");
        telemetry.update();
        initHardware();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        /*
        // Go forward away from wall for 6 inches
        if (opModeIsActive()) {
            driveFor(.25, true);
        }
        // Turn left 90 degrees
        if (opModeIsActive()) {
            turnFor(.50, false);
        }
        // Go forward 2 feet (24 inches)
        if (opModeIsActive()) {
            driveFor(1, true);
        }
         */
        // Bring slide up
        if(opModeIsActive()) {
            moveSlide(2, true);
        }
        // 32 inches
        if(opModeIsActive()) {
            driveFor(1.5, SPEED);
        }
        // Bring slide down
        if(opModeIsActive()) {
            moveSlide(1.9, false);
        }
        // Back
        if(opModeIsActive()) {
            driveFor(1.5, -SPEED*3);
        }
    }

    /**
     * A simple method used to make our robot reverse or go forward.
     *
     * @param time The amount of time in seconds to drive for
     */
    private void driveFor(double time, double speed) {
        // Determine the direction and power to set
        leftDrive.setPower(speed);
        rightDrive.setPower(speed);
        // Reset the timeout time and start motion
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time));
        // Stop all motion
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    /**
     * A simple method used to turn our robot.
     *
     * @param time The amount of time in seconds to execute a turn for
     * @param right True to turn right, false to turn left
     */
    private void turnFor(double time, boolean right) {
        // Determine the direction and power to set
        if (right) {
            leftDrive.setPower(SPEED);
            rightDrive.setPower(-SPEED);
        } else {
            leftDrive.setPower(-SPEED);
            rightDrive.setPower(SPEED);
        }
        // Reset the timeout time and start motion
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time));
        // Stop all motion
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    private void moveSlide(double time, boolean up) {
        // Determine the direction and power to set
        if (up) {
            theSlide.setPower(-SPEED);
        } else {
            theSlide.setPower(SPEED);
        }
        // Reset the timeout time and start motion
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time));
        // Stop all motion
        theSlide.setPower(0);
    }

    /**
     * Simply initializes our hardware from the FTC config into variables.
     */
    private void initHardware() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        theSlide = hardwareMap.get(DcMotor.class, "the_slide");

        // Our robot needs the motor on one side to be reversed to drive forward
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Ensure to not run with encoder
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Log that init hardware is finished
        telemetry.log().clear();
        telemetry.log().add("Init. hardware finished.");
        telemetry.clear();
        telemetry.update();
    }

}
