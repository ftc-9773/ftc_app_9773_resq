package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;

/**
 * Created by pranavb on 10/8/15.
 */
public class mecanumWheels extends LinearOpMode {
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor rearLeft;
    DcMotor rearRight;
    ColorSensor colSensor;
    DeviceInterfaceModule dim;
    static final int LED_CHANNEL = 0;
    float frontLeftPwr;
    float frontRightPwr;
    float rearLeftPwr;
    float rearRightPwr;

    /*public void lineFollow(double kP, double minRLI, double maxRLI){
        while(true) {
            double redVal = colSensor.red();
            double greenVal = colSensor.green();
            double blueVal = colSensor.blue();

            double rliVal = redVal + greenVal + blueVal;
        }
    }*/

    @Override
    public void runOpMode() throws InterruptedException{
        frontLeft = hardwareMap.dcMotor.get("fMotorL");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight = hardwareMap.dcMotor.get("fMotorR");
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        rearLeft = hardwareMap.dcMotor.get("rMotorL");
        rearRight = hardwareMap.dcMotor.get("rMotorR");
        dim = hardwareMap.deviceInterfaceModule.get("dim");
        dim.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);
        colSensor = hardwareMap.colorSensor.get("colorSensor");

        boolean colEnabled = true;
        dim.setDigitalChannelState(LED_CHANNEL, colEnabled);

        waitOneFullHardwareCycle();

        waitForStart();

        while(opModeIsActive()) {
            float strafeDirection = -gamepad1.left_stick_x;
            float throttle1 = -gamepad1.left_stick_y;
            float throttle2 = gamepad1.right_stick_y;
            float direction = -gamepad1.right_stick_x;

            strafeDirection = Range.clip(strafeDirection, -1, 1);
            throttle1 = Range.clip(throttle1, -1, 1);
            throttle2 = Range.clip(throttle2, -1, 1);
            direction = Range.clip(direction, -1, 1);

            frontLeftPwr = -(Range.clip(throttle1 + strafeDirection, -1, 1) - Range.clip(throttle2 + direction, -1, 1));
            frontRightPwr = -(Range.clip(strafeDirection - throttle1, -1, 1) + Range.clip(throttle2 - direction, -1, 1));
            rearLeftPwr = Range.clip(strafeDirection - throttle1, -1, 1) + Range.clip(throttle2 + direction, -1, 1);
            rearRightPwr = Range.clip(throttle1 + strafeDirection, -1, 1) - Range.clip(throttle2 - direction, -1, 1);

            frontLeft.setPower(Range.clip(frontLeftPwr, -1, 1));
            rearRight.setPower(Range.clip(rearRightPwr, -1, 1));
            frontRight.setPower(Range.clip(frontRightPwr, -1, 1));
            rearLeft.setPower(Range.clip(rearLeftPwr, -1, 1));

            double rValue = colSensor.red();
            double gValue = colSensor.green();
            double bValue = colSensor.blue();
            double rliValue = rValue + gValue + bValue;

            telemetry.addData("Red ", rValue);
            telemetry.addData("Green ", gValue);
            telemetry.addData("Blue ", bValue);
            telemetry.addData("Sum ", rliValue);
        }
    }

    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);
        if (index < 0) {
            index = -index;
        } else if (index > 16) {
            index = 16;
        }

        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        return dScale;
    }
}
