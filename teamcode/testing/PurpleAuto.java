/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Disabled
@Autonomous(name = "PurpleAuto", group = "Linear Opmode")
public class PurpleAuto extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    // The speed for the drive motors to operate at during autonomous
    private static final double SPEED = 0.3;

    // Declare hardware
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor theSlide = null;

    // Define class members
    private Servo leftClaw = null;
    private Servo rightClaw = null;

    private boolean lastResetState = false;
    private boolean curResetState = false;

    /**
     * In this sample, for illustration purposes we use two interfaces on the one gyro object.
     * That's likely atypical: you'll probably use one or the other in any given situation,
     * depending on what you're trying to do. {@link IntegratingGyroscope} (and it's base interface,
     * {@link Gyroscope}) are common interfaces supported by possibly several different gyro
     * implementations. {@link ModernRoboticsI2cGyro}, by contrast, provides functionality that
     * is unique to the Modern Robotics gyro sensor.
     */
    private IntegratingGyroscope gyro = null;
    private ModernRoboticsI2cGyro modernRoboticsI2cGyro = null;

    // A timer helps provide feedback while calibration is taking place
    private ElapsedTime timer = new ElapsedTime();

    private static final double SERVO_MIN_POS = 0.0; // Minimum rotational position
    private static final double SERVO_MAX_POS = 1.0; // Maximum rotational position
    private static final double SERVO_HALFWAY_POSITION = (SERVO_MAX_POS - SERVO_MIN_POS) / 2; // Start at halfway position
//https://www.andymark.com/products/neverest-classic-40-gearmotor
    private static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    private static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.14);
    // Divide for 2:1 gears
    private static final double ACTUAL_COUNTS_PER_INCH = COUNTS_PER_INCH / DRIVE_GEAR_REDUCTION;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        initHardware();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        /*
        // Go five inches forward
        if(opModeIsActive()) {
            encoderDrive(.5, 5, 5, 5);
        }
        // Turn right 11 inches (12 inches is just past 180 degrees)
        if(opModeIsActive()) {
            encoderDrive(.5, 11, -11, 4);
        }
        // Forward ten inches
        if(opModeIsActive()) {
            encoderDrive(.5, 10, 10, 5);
        }
        // Turn left 5.5 inches (12 inches is just past 180 degrees)
        if(opModeIsActive()) {
            encoderDrive(.5, -5.5, 5.5, 4);
        }
        // Open the claws
        if(opModeIsActive()) {
            openClaw(true);
        }
        */

        //bring slide up to make it over the foundation later
        if (opModeIsActive()) {
            moveSlide(2,true);
        }
        //Go forward 29 inches at half speed to move towards the foundation
        if (opModeIsActive()) {
            encoderDrive(.5, 29, 29, 5);
        }
        //we found that -11 and 11 create about a 90 degree turn
        //we turn 90 degrees to the left to turn towards the blue foundation
        if (opModeIsActive()) {
            encoderDrive(.5, -11, 11, 5);
        }
        //go forward 13 inches to move robot with the center of the foundation
        if (opModeIsActive()) {
            encoderDrive(.5,13,13,5);
        }
        //turn right 90 degrees to align with the foundation
        if(opModeIsActive()) {
            encoderDrive(.5,11,-11,5);
        }
        //move another 6 inches forward to get the catcher over the foundation
        if (opModeIsActive()) {
            encoderDrive(.5, 7, 7, 5);
        }
        //move the slide down in order for it to get the slide
        if (opModeIsActive()){
            moveSlide(1.75,false);
        }
        //turn 90 degrees left in order to  move the foundation **WE MIGHT HAVE TO CHANGE THIS**
        if (opModeIsActive()) {
            encoderDrive(1, -38, -38, 5);
        }
        //Havent tested this next part yet
        //Move slide up in order to get it off the foundation
        if (opModeIsActive()) {
            moveSlide(1, true);
        }
        //move it forward in order to get it against the wall into the base
        if (opModeIsActive()) {
            encoderDrive(.5, 11, -11, 5);
        }

        //go back roughly 72 inches in order to park under the bridge
        if (opModeIsActive()) {
            encoderDrive(.5, 36, 36, 5);
        }

        // wait 50 seconds to view telemetry
        //REMEMBER TO TAKE THIS OUR BEFORE COMPETITION
        //sleep(50000);
       /* gyroCheck();
        sleep(5000);
        // Turn left 90 degrees (11 inches is about 90 degrees)
        if (opModeIsActive()) {
           // encoderDrive(.5, -11, 11, 4);
            turnToHeading(90, .5, false);
        }
        gyroCheck();
        sleep(5000);
        // Go forward 2 feet (24 inches)
        if (opModeIsActive()) {
            encoderDrive(.5, 24, 24, 4);
        }
        gyroCheck();
        sleep(5000);
        // Go forward 2 feet (24 inches)
        if (opModeIsActive()) {
            //  encoderDrive(.5, 22, -22, 4);
           // turnToHeading(270, .5, true);
        }
        gyroCheck();
        sleep(5000);

        // Open claw
        if (opModeIsActive()) {
            openClaw(true);
        }
*/
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    private void encoderDrive(double speed,
                              double leftInches, double rightInches,
                              double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftDrive.getCurrentPosition() + (int) (leftInches * ACTUAL_COUNTS_PER_INCH);
            newRightTarget = rightDrive.getCurrentPosition() + (int) (rightInches * ACTUAL_COUNTS_PER_INCH);
            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);


            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftDrive.setPower(Math.abs(speed));
            rightDrive.setPower(Math.abs(speed));

            int leftTicks = leftDrive.getCurrentPosition();
            int rightTicks = rightDrive.getCurrentPosition();
            telemetry.addData("Start Position", "Running at %7d :%7d", leftTicks, rightTicks);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftDrive.isBusy() || rightDrive.isBusy()));
            // Display it for the driver.
            telemetry.addData("Target Position", "Running to %7d :%7d", newLeftTarget, newRightTarget);
            telemetry.addData("End Position", "Running at %7d :%7d", leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
            telemetry.addData("Distance Traveled", "Running at %7d :%7d",leftDrive.getCurrentPosition()-leftTicks, rightDrive.getCurrentPosition()-rightTicks);
            telemetry.update();

            // Stop all motion;
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    /**
     * This is not working
     * <p>
     * Turns the robot to a desired angle and set speed and direction
     * <p>
     * Heading values:
     *      0
     * 90       270
     *     180
     *
     * @param degree    Position on the heading to turn to
     * @param speed     Wheel speed
     * @param turnRight True if you want to turn right to get to the angle and false to turn left
     */
    private void turnToHeading(int degree, double speed, boolean turnRight) {
        if (turnRight) {
            // Set the power for right turn
            leftDrive.setPower(speed);
            rightDrive.setPower(-speed);
            while (modernRoboticsI2cGyro.getHeading() > degree) {
                // Continue doing nothing until at angle
            }
        } else {
            // Set the power for left turn
            leftDrive.setPower(-speed);
            rightDrive.setPower(speed);
            while (modernRoboticsI2cGyro.getHeading() < degree) {
                // Continue doing nothing until at angle
            }
        }
        // Stop all motion
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        // Turn off RUN_TO_POSITION
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * does String.format("%d", rawValue)
     *
     * @param rawValue
     * @return
     */
    private String formatRaw(int rawValue) {
        return String.format("%d", rawValue);
    }

    /**
     * does  String.format("%.3f", rate)
     *
     * @param rate
     * @return
     */
    private String formatRate(float rate) {
        return String.format("%.3f", rate);
    }

    private void initHardware() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftClaw = hardwareMap.get(Servo.class, "left_claw");
        rightClaw = hardwareMap.get(Servo.class, "right_claw");
        theSlide = hardwareMap.get(DcMotor.class, "the_slide");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        theSlide.setDirection(DcMotor.Direction.FORWARD);
        // Setup motor of encoders
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        theSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        theSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Get a reference to a Modern Robotics gyro object. We use several interfaces
        // on this object to illustrate which interfaces support which functionality.
        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope) modernRoboticsI2cGyro;
        // If you're only interested int the IntegratingGyroscope interface, the following will suffice.
        // gyro = hardwareMap.get(IntegratingGyroscope.class, "gyro");
        // A similar approach will work for the Gyroscope interface, if that's all you need.

        // Start calibrating the gyro. This takes a few seconds and is worth performing
        // during the initialization phase at the start of each opMode.
        telemetry.log().add("Gyro Calibrating. Do Not Move!");
        modernRoboticsI2cGyro.calibrate();

        // Wait until the gyro calibration is complete
        timer.reset();
        while (!isStopRequested() && modernRoboticsI2cGyro.isCalibrating()) {
            telemetry.addData("calibrating", "%s", Math.round(timer.seconds()) % 2 == 0 ? "|.." : "..|");
            telemetry.update();
            sleep(50);
        }

        telemetry.log().clear();
        telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear();
        telemetry.update();
    }

    /**
     * Sets claws to opened or closed.
     *
     * @param openClaw True to have open, false to have closed
     */
    private void openClaw(boolean openClaw) {
        if (openClaw) {
            leftClaw.setPosition(SERVO_MAX_POS);
            rightClaw.setPosition(SERVO_MIN_POS);
        } else {
            // Set the servo to the new position and pause
            leftClaw.setPosition(SERVO_MIN_POS);
            rightClaw.setPosition(SERVO_MAX_POS);
        }
        telemetry.addData("Servo claws", "left (%.2f), right (%.2f)", leftClaw.getPosition(), rightClaw.getPosition()); // Send servo position
    }

    /**
     * Checks gyro and outputs to telemetry
     */
    private void gyroCheck() {
        // If the A and B buttons are pressed just now, reset Z heading.
        curResetState = (gamepad1.a && gamepad1.b);
        if (curResetState && !lastResetState) {
            modernRoboticsI2cGyro.resetZAxisIntegrator();
        }
        lastResetState = curResetState;

        // The raw() methods report the angular rate of change about each of the
        // three axes directly as reported by the underlying sensor IC.
        int rawX = modernRoboticsI2cGyro.rawX();
        int rawY = modernRoboticsI2cGyro.rawY();
        int rawZ = modernRoboticsI2cGyro.rawZ();
        int heading = modernRoboticsI2cGyro.getHeading();
        int integratedZ = modernRoboticsI2cGyro.getIntegratedZValue();

        // Read dimensionalized data from the gyro. This gyro can report angular velocities
        // about all three axes. Additionally, it internally integrates the Z axis to
        // be able to report an absolute angular Z orientation.
        AngularVelocity rates = gyro.getAngularVelocity(AngleUnit.DEGREES);
        float zAngle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        // Read administrative information from the gyro
        int zAxisOffset = modernRoboticsI2cGyro.getZAxisOffset();
        int zAxisScalingCoefficient = modernRoboticsI2cGyro.getZAxisScalingCoefficient();

        telemetry.addLine()
                .addData("dx", formatRate(rates.xRotationRate))
                .addData("dy", formatRate(rates.yRotationRate))
                .addData("dz", "%s deg/s", formatRate(rates.zRotationRate));
        telemetry.addData("angle", "%s deg", String.format("%.3f", zAngle));
        telemetry.addData("heading", "%3d deg", heading);
        telemetry.addData("integrated Z", "%3d", integratedZ);
        /*telemetry.addLine()
                .addData("rawX", formatRaw(rawX))
                .addData("rawY", formatRaw(rawY))
                .addData("rawZ", formatRaw(rawZ));

         */
        telemetry.addLine().addData("z offset", zAxisOffset).addData("z coeff", zAxisScalingCoefficient);
        telemetry.update();
    }
    private void moveSlide(double time, boolean up) {
        int slideTick = theSlide.getCurrentPosition();
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
        telemetry.addData("Distance Slide Traveled", "Running at %7d",theSlide.getCurrentPosition()-slideTick);
        telemetry.update();
    }
}
