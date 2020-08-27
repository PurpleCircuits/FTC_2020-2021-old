package org.firstinspires.ftc.teamcode.TeamCodeForTournament.ReadyCode;

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
@Autonomous(name = "L36Bridge", group = "Linear Opmode")
public class L36Bridge extends LinearOpMode {

    // The speed for the drive motors to operate at during autonomous
    private static final double SPEED = 0.3;

    // Declare hardware
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    // Used for determining how long something has ran
    private ElapsedTime runtime = new ElapsedTime();

    /**
     * This is the entry of our Op Mode.
     */
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "L36Bridge");
        telemetry.update();
        initHardware();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Go forward away from wall for 36 inches
        if (opModeIsActive()) {
            driveFor(1, true);
        }
        // Turn right 90 degrees
        if (opModeIsActive()) {
            turnFor(.50, true);
        }
        // Go forward 2 feet (24 inches)
        if (opModeIsActive()) {
            driveFor(1, true);
        }
    }

    /**
     * A simple method used to make our robot reverse or go forward.
     *
     * @param time The amount of time in seconds to drive for
     * @param forward True to go forward, false to go background
     */
    private void driveFor(double time, boolean forward) {
        // Determine the direction and power to set
        if (forward) {
            leftDrive.setPower(SPEED);
            rightDrive.setPower(SPEED);
        } else {
            leftDrive.setPower(-SPEED);
            rightDrive.setPower(-SPEED);
        }
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

    /**
     * Simply initializes our hardware from the FTC config into variables.
     */
    private void initHardware() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

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
