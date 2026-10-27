package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;


public class Robot {
    // Declare hardware members
    private DcMotor frontLeftWheel = null;
    private DcMotor backLeftWheel = null;
    private DcMotor frontRightWheel = null;
    private DcMotor backRightWheel = null;
    private DcMotorEx pollenLauncher = null;
    private DcMotorEx nectarLauncher = null;
    private Servo nectarGate = null;
    private Servo pollenGate = null;
    private LinearOpMode opMode;
    private IMU imu;
    // Constructor
    public Robot(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    // Initialize standard Hardware interfaces
    public void init(HardwareMap hardwareMap) {

        // Map devices to the names configured on the Driver Station / Control Hub
        frontLeftWheel  = hardwareMap.get(DcMotor.class, "front_left_wheel");
        backLeftWheel = hardwareMap.get(DcMotor.class, "back_left_wheel");
        frontRightWheel = hardwareMap.get(DcMotor.class, "front_right_wheel");
        backRightWheel = hardwareMap.get(DcMotor.class, "back_right_wheel");
        nectarGate = hardwareMap.get(Servo.class, "nectar_gate");
        pollenGate = hardwareMap.get(Servo.class, "pollen_gate");

        // Set motor directions (adjust according to your drivetrain orientation)
        frontLeftWheel.setDirection(DcMotor.Direction.FORWARD);
        backLeftWheel.setDirection(DcMotor.Direction.FORWARD);
        frontRightWheel.setDirection(DcMotor.Direction.REVERSE);
        backRightWheel.setDirection(DcMotor.Direction.REVERSE);

        // Set zero-power behavior
        frontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

    // Helper methods keep OpModes clean
    public void setDrivePower(double forward, double right, double rotate) {

        double max;

        double frontLeftPower  = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backLeftPower   = forward - right + rotate;
        double backRightPower  = forward + right - rotate;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;

            // Send calculated power to wheels
            frontLeftWheel.setPower(frontLeftPower);
            frontRightWheel.setPower(frontRightPower);
            backLeftWheel.setPower(backLeftPower);
            backRightWheel.setPower(backRightPower);
        }
    }
    public void setPollenLauncherPower(double power) {
        pollenLauncher.setPower(power);
    }

    public void setNectarLauncher(double power) {
        nectarLauncher.setPower(power);
    }
    public void nectarGateOpen(double position) {
        nectarGate.setPosition(position);
    }
    public void pollenGateOpen(double position) {
        pollenGate.setPosition(position);
    }
    public void setRelativeDrive(double forward, double right, double rotate, boolean resetView, boolean robotView) {
        opMode.telemetry.addLine("Press A to reset Yaw");
        opMode.telemetry.addLine("Hold left bumper to drive in robot relative");
        opMode.telemetry.addLine("The left joystick sets the robot direction");
        opMode.telemetry.addLine("Moving the right joystick left and right turns the robot");

        // If you press the A button, then you reset the Yaw to be zero from the way
        // the robot is currently pointing
        if (resetView) {
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
