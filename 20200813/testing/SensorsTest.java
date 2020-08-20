package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Most of this code was copied from the FTC examples, but we tweaked it for our purposes.
 *
 * This makes the robot drive forward and turn left and go forward, parking under the bridge during
 * autonomous gaining us 5 points.
 */
@Autonomous(name = "L36Bridge", group = "Linear Opmode")
public class SensorsTest extends LinearOpMode {

    // The speed for the drive motors to operate at during autonomous
    private static final double SPEED = 0.3;

    // Declare hardware
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    BNO055IMU imu;

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
        if (opModeIsActive())  {
            turnFor(.5, false);
        }
        // Go forward 2 feet (24 inches)
        if (opModeIsActive()) {
            driveFor(1, false);
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
    public void turnLeft(double turnAngle, double timeoutS) {
        sleep(500);
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double speed=.5;
        double oldDegreesLeft=turnAngle;
        double scaledSpeed=speed;
        double targetHeading=angles.firstAngle+turnAngle;
        double oldAngle=angles.firstAngle;
        if(targetHeading<-180) {targetHeading+=360;}
        if(targetHeading>180){targetHeading-=360;}
        double degreesLeft =
                ((int)(Math.signum(angles.firstAngle-targetHeading)+1)/2)
                        *(360-Math.abs(angles.firstAngle-targetHeading))
                        +
                        (int)(Math.signum(targetHeading-angles.firstAngle)+1)
                                /2*Math.abs(angles.firstAngle-targetHeading);
        runtime.reset();
        while(opModeIsActive() &&
                runtime.seconds() < timeoutS &&
                degreesLeft>1&&
                oldDegreesLeft-degreesLeft>=0) { //check to see if we overshot target
            scaledSpeed=degreesLeft/(100+degreesLeft)*speed;
            if(scaledSpeed>1){scaledSpeed=.1;}

            leftDrive.setPower(scaledSpeed*1.3); //extra power to back wheels
            rightDrive.setPower(-1*scaledSpeed*1.3); //due to extra weight
            //robot.leftFront.setPower(scaledSpeed);
            //robot.rightFront.setPower(-1*scaledSpeed);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            oldDegreesLeft=degreesLeft;
            degreesLeft = ((int)(Math.signum(angles.firstAngle-targetHeading)+1)/2)*(360-Math.abs(angles.firstAngle-targetHeading))+(int)(Math.signum(targetHeading-angles.firstAngle)+1)/2*Math.abs(angles.firstAngle-targetHeading);
            if(Math.abs(angles.firstAngle-oldAngle)<1){speed*=1.1;} //bump up speed to wheels in case robot stalls before reaching target
            oldAngle=angles.firstAngle;
        }
        sleep(250); //small pause at end of turn
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


        // We are expecting the IMU to be attached to an I2C port on a Core Device Interface Module and named "imu".
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag     = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Log that init hardware is finished
        telemetry.log().clear();
        telemetry.log().add("Init. hardware finished.");
        telemetry.clear();
        telemetry.update();
    }

}
