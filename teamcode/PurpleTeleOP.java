package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/**
 * Our TeleOp for controlling our motors, linear slide and claws.
 * We also can adjust the speed of our robot.
 */

@TeleOp(name="PurpleTeleOp", group="Linear Opmode")
public class PurpleTeleOP extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // Declare our hardware
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor theSlide = null;
    private Servo leftClaw = null;
    private Servo rightClaw = null;
    private IntegratingGyroscope gyro = null;
    private Gyroscope modernRoboticsI2cGyro = null;

    private boolean lastResetState = false;
    private boolean curResetState  = false;

    // A timer helps provide feedback while calibration is taking place
    private ElapsedTime timer = new ElapsedTime();

    // Settings for our servo
    private static final double SERVO_MIN_POS = 0.0; // Minimum rotational position
    private static final double SERVO_MAX_POS = 1.0; // Maximum rotational position
    private static final double SERVO_HALFWAY_POSITION = (SERVO_MAX_POS - SERVO_MIN_POS) / 2; // Start at halfway position

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        initHardware();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            driveAction();

            clawAction();

            //gyroCheck();

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }

    /**
     * does String.format("%d", rawValue)
     * @param rawValue
     * @return
     */
    private String formatRaw(int rawValue) {
        return String.format("%d", rawValue);
    }
    /**
     * does  String.format("%.3f", rate)
     * @param rate
     * @return
     */
     private String formatRate(float rate) {
        return String.format("%.3f", rate);
     }

     private void initHardware(){
         // Initialize the hardware variables. Note that the strings used here as parameters
         // to 'get' must correspond to the names assigned during the robot configuration
         // step (using the FTC Robot Controller app on the phone).
         leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
         rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
         theSlide = hardwareMap.get(DcMotor.class, "the_slide");
         leftClaw = hardwareMap.get(Servo.class, "left_claw");
         rightClaw = hardwareMap.get(Servo.class, "right_claw");

         // Most robots need the motor on one side to be reversed to drive forward
         leftDrive.setDirection(DcMotor.Direction.FORWARD);
         rightDrive.setDirection(DcMotor.Direction.REVERSE);

         // Get a reference to a Modern Robotics gyro object. We use several interfaces
         // on this object to illustrate which interfaces support which functionality.
         //modernRoboticsI2cGyro = hardwareMap.get(Gyroscope.class, "gyro");
         //gyro = (IntegratingGyroscope)modernRoboticsI2cGyro;
         // If you're only interested int the IntegratingGyroscope interface, the following will suffice.
         // gyro = hardwareMap.get(IntegratingGyroscope.class, "gyro");
         // A similar approach will work for the Gyroscope interface, if that's all you need.

         // Start calibrating the gyro. This takes a few seconds and is worth performing
         // during the initialization phase at the start of each opMode.
         telemetry.log().add("Gyro Calibrating. Do Not Move!");
         //modernRoboticsI2cGyro.calibrate();

         // Wait until the gyro calibration is complete
         /*timer.reset();
         while (!isStopRequested() && modernRoboticsI2cGyro.isCalibrating())  {
             telemetry.addData("calibrating", "%s", Math.round(timer.seconds())%2==0 ? "|.." : "..|");
             telemetry.update();
             sleep(50);
         }
        */
         telemetry.log().clear();
         telemetry.log().add("Gyro Calibrated. Press Start.");
         telemetry.clear();
         telemetry.update();
     }

    /**
     * Sets servo position depending on button input, A = Middle B = Grab X = Open
     */
    private void clawAction() {
         // Move claws to half way
         if(gamepad2.a) {
             // Set the servo to the new position and pause
             leftClaw.setPosition(SERVO_HALFWAY_POSITION);
             rightClaw.setPosition(SERVO_HALFWAY_POSITION);
         }
         // If b is pressed than set to grab stateSS
         if(gamepad2.b) {
             // Set the servo to the new position and pause
             leftClaw.setPosition(SERVO_MIN_POS);
             rightClaw.setPosition(SERVO_MAX_POS);
         }
         // Opens Claw all the way
         if(gamepad2.x) {
             leftClaw.setPosition(SERVO_MAX_POS);
             rightClaw.setPosition(SERVO_MIN_POS);
         }

         // Control the linear slide
         double slide = gamepad2.left_stick_y;
         theSlide.setPower(slide/2);

         telemetry.addData("Servo claws", "left (%.2f), right (%.2f)", leftClaw.getPosition(), rightClaw.getPosition()); // Send servo position
    }

    /**
     * Calculates drive power for POV and outputs power to telemetry
     */
    private void driveAction(){
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower = 0;
        double rightPower = 0;

        if (gamepad1.left_bumper) {
            // Set to tank mode
            leftPower=-gamepad1.left_stick_y;
            rightPower=-gamepad1.right_stick_y;
        } else {
            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            leftPower = Range.clip(drive + turn, -1.0, 1.0);
            rightPower = Range.clip(drive - turn, -1.0, 1.0);
        }
        // Slow down the robot by factor 5
        if (!gamepad1.right_bumper) {
            leftPower = leftPower/5;
            rightPower = rightPower/5;
        }
        // Send calculated power to wheels
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        // Telemetry wont be updated until update is called
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

    /**
     * Checks gyro and outputs to telemetry
     *
    private void gyroCheck(){
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
        telemetry.addLine()
                .addData("rawX", formatRaw(rawX))
                .addData("rawY", formatRaw(rawY))
                .addData("rawZ", formatRaw(rawZ));
        telemetry.addLine().addData("z offset", zAxisOffset).addData("z coeff", zAxisScalingCoefficient);
    }
/* */
}