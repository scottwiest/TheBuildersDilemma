package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Robot {
    // Declare hardware members
    private DcMotor frontLeftWheel = null;
    private DcMotor backLeftWheel = null;
    private DcMotor frontRightWheel = null;
    private DcMotor backRightWheel = null;
    private DcMotorEx pollenLauncher = null;
    private DcMotorEx nectarLauncher = null;
    private DcMotor transferIntake = null;
    private Servo nectarGate = null;
    private Servo pollenGate = null;
    private LinearOpMode opMode;
    private IMU imu;
    private ElapsedTime runtime = new ElapsedTime();
    static final double COUNTS_PER_TIRE_REV = 537.7;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_TIRE_REV) / (WHEEL_DIAMETER_INCHES * 3.14159);

    // Constructor
    public Robot(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    // Initialize standard Hardware interfaces
    public void init(HardwareMap hardwareMap) {

        // Map devices to the names configured on the Driver Station / Control Hub
        frontLeftWheel = hardwareMap.get(DcMotor.class, "front_left_wheel");
        backLeftWheel = hardwareMap.get(DcMotor.class, "back_left_wheel");
        frontRightWheel = hardwareMap.get(DcMotor.class, "front_right_wheel");
        backRightWheel = hardwareMap.get(DcMotor.class, "back_right_wheel");
        pollenLauncher = hardwareMap.get(DcMotorEx.class, "pollen_launcher");
        nectarLauncher = hardwareMap.get(DcMotorEx.class, "nectar_launcher");
        transferIntake = hardwareMap.get(DcMotor.class, "transfer_intake");
        nectarGate = hardwareMap.get(Servo.class, "nectar_gate");
        pollenGate = hardwareMap.get(Servo.class, "pollen_gate");

        // Set motor directions (adjust according to your drivetrain orientation)
        frontLeftWheel.setDirection(DcMotor.Direction.FORWARD);
        backLeftWheel.setDirection(DcMotor.Direction.FORWARD);
        frontRightWheel.setDirection(DcMotor.Direction.REVERSE);
        backRightWheel.setDirection(DcMotor.Direction.REVERSE);
        pollenLauncher.setDirection(DcMotorEx.Direction.FORWARD);
        nectarLauncher.setDirection(DcMotorEx.Direction.REVERSE);
        transferIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set zero-power behavior
        frontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Reset and stop motors
        frontLeftWheel.setPower(0);
        backLeftWheel.setPower(0);
        frontRightWheel.setPower(0);
        backRightWheel.setPower(0);

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    public void setRunUsingEncoder() {
        frontLeftWheel.setMode(RunMode.STOP_AND_RESET_ENCODER);
        frontRightWheel.setMode(RunMode.STOP_AND_RESET_ENCODER);
        backLeftWheel.setMode(RunMode.STOP_AND_RESET_ENCODER);
        backRightWheel.setMode(RunMode.STOP_AND_RESET_ENCODER);
        pollenLauncher.setMode(RunMode.STOP_AND_RESET_ENCODER);
        nectarLauncher.setMode(RunMode.STOP_AND_RESET_ENCODER);

        frontLeftWheel.setMode(RunMode.RUN_USING_ENCODER);
        frontRightWheel.setMode(RunMode.RUN_USING_ENCODER);
        backLeftWheel.setMode(RunMode.RUN_USING_ENCODER);
        backRightWheel.setMode(RunMode.RUN_USING_ENCODER);
        pollenLauncher.setMode(RunMode.RUN_USING_ENCODER);
        nectarLauncher.setMode(RunMode.RUN_USING_ENCODER);
    }
    // Helper methods keep OpModes clean
    public void setDrivePower(double forward, double right, double rotate) {

        double max;

        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backLeftPower = forward - right + rotate;
        double backRightPower = forward + right - rotate;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        double scalingFactor = 0.65;
        // Send calculated power to wheels
        frontLeftWheel.setPower(frontLeftPower * scalingFactor);
        frontRightWheel.setPower(frontRightPower * scalingFactor);
        backLeftWheel.setPower(backLeftPower * scalingFactor);
        backRightWheel.setPower(backRightPower * scalingFactor);
    }
    public void runDriveInstructions(
            double speed,
            double leftInches,
            double rightInches,
            double timeoutS
    ) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the OpMode is still active
        if (opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget =
                    frontLeftWheel.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightFrontTarget =
                    frontRightWheel.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newLeftBackTarget = backLeftWheel.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightBackTarget =
                    backRightWheel.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            frontLeftWheel.setTargetPosition(newLeftFrontTarget);
            frontRightWheel.setTargetPosition(newRightFrontTarget);
            backLeftWheel.setTargetPosition(newLeftBackTarget);
            backRightWheel.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            frontLeftWheel.setMode(RunMode.RUN_TO_POSITION);
            frontRightWheel.setMode(RunMode.RUN_TO_POSITION);
            backLeftWheel.setMode(RunMode.RUN_TO_POSITION);
            backRightWheel.setMode(RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            frontLeftWheel.setPower(Math.abs(speed));
            frontRightWheel.setPower(Math.abs(speed));
            backLeftWheel.setPower(Math.abs(speed));
            backRightWheel.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (frontLeftWheel.isBusy() && frontRightWheel.isBusy() && backLeftWheel.isBusy()
                            && backRightWheel.isBusy())) {
                opMode.sleep(10);
            }

            // Stop all motion;
            frontLeftWheel.setPower(0);
            frontRightWheel.setPower(0);
            backLeftWheel.setPower(0);
            backRightWheel.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeftWheel.setMode(RunMode.RUN_USING_ENCODER);
            frontRightWheel.setMode(RunMode.RUN_USING_ENCODER);
            backLeftWheel.setMode(RunMode.RUN_USING_ENCODER);
            backRightWheel.setMode(RunMode.RUN_USING_ENCODER);

            opMode.sleep(100);   // pause after each move.
        }
    }
        public void leftStrafe() {
            frontRightWheel.setPower(1);
            frontLeftWheel.setPower(-1);
            backRightWheel.setPower(-1);
            backLeftWheel.setPower(1);
        }

        public void rightStrafe() {
            frontRightWheel.setPower(-1);
            frontLeftWheel.setPower(1);
            backRightWheel.setPower(1);
            backLeftWheel.setPower(-1);
        }


    public void setPollenLauncher(double power) {
        pollenLauncher.setPower(power);
    }

    public void setNectarLauncher(double power) {
        nectarLauncher.setPower(power);
    }
    public void setTransferIntake(double power) {
        transferIntake.setPower(power);
    }
    public void nectarGateOpen() {
        // 0.5 is equal to 90 degrees
        nectarGate.setPosition(0.3);
    }

    public void pollenGateOpen() {
        // 0.22 is equal to 40 degrees
        pollenGate.setPosition(0);
    }

    public void pollenGateClose() {
        pollenGate.setPosition(0.2);
    }

    public void nectarGateClose() {
        nectarGate.setPosition (0.1);
    }

    public void setPollenGate(double position){ 
        pollenGate.setPosition(position);
    }
    public void setNectarGate(double position){
        nectarGate.setPosition(position);
    }

    public void setRelativeDrive(
            double forward,
            double right,
            double rotate,
            boolean resetRotation,
            boolean robotView
    ) {
        // If you press the A button, then you reset the Yaw to be zero from the way
        // the robot is currently pointing
        if (resetRotation) {
            imu.resetYaw();
        }
        // If you press the left bumper, you get a drive from the point of view of the robot
        // (much like driving an RC vehicle)
        if (robotView) {
            setDrivePower(forward, right, rotate);
        } else {
            driveFieldRelative(forward, right, rotate);
        }
    }

    private void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        setDrivePower(newForward, newRight, rotate);
    }
}
