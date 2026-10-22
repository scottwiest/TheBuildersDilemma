package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {
    // Declare hardware members
    public DcMotor frontLeftWheel = null;
    public DcMotor backLeftWheel = null;
    public DcMotor frontRightWheel = null;
    public DcMotor backRightWheel = null;
    public DcMotorEx pollenLauncher = null;
    public DcMotorEx nectarLauncher = null;
    public Servo nectarGate = null;
    public Servo pollenGate = null;
    private LinearOpMode opMode;

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
    }

    // Helper methods keep OpModes clean
    public void setDrivePower(double axial, double lateral, double yaw) {

        double max;

        double frontLeftPower  = axial + lateral + yaw;
        double frontRightPower = axial - lateral - yaw;
        double backLeftPower   = axial - lateral + yaw;
        double backRightPower  = axial + lateral - yaw;

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
}
