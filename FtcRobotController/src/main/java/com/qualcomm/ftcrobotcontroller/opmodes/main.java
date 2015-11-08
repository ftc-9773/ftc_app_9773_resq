package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftcrobotcontroller.opmodes.DriveSystem.Wheel.MecanumWheel;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by pb8xe_000 on 10/31/2015.
 */
public class main extends LinearOpMode {
    MecanumWheel mecanumWheel;
    DriveSystem mecanumDriveSystem;
    ColorSensor colSensor;
    DeviceInterfaceModule dim;
    static final int LED_CHANNEL = 0;

    /*public void lineFollow(double kP, double minRLI, double maxRLI){
        while(true) {
            double redVal = colSensor.red();
            double greenVal = colSensor.green();
            double blueVal = colSensor.blue();

            double rliVal = redVal + greenVal + blueVal;
        }
    }*/
    @Override
    public void runOpMode() throws InterruptedException {
        dim = hardwareMap.deviceInterfaceModule.get("dim");
        dim.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);
        colSensor = hardwareMap.colorSensor.get("colorSensor");

        boolean colEnabled = true;
        dim.setDigitalChannelState(LED_CHANNEL, colEnabled);


        waitOneFullHardwareCycle();

        waitForStart();

        while (opModeIsActive()) {
            float gamepadStrafeDirection = -gamepad1.left_stick_x;
            float gamepadStrafeThrottle = -gamepad1.left_stick_y;
            float gamepadTurnThrottle = gamepad1.right_stick_y;
            float gamepadTurnDirection = -gamepad1.right_stick_x;
            gamepadStrafeThrottle = Range.clip(gamepadStrafeThrottle, -1, 1);
            gamepadStrafeDirection = Range.clip(gamepadStrafeThrottle, -1, 1);
            gamepadTurnThrottle = Range.clip(gamepadTurnThrottle, -1, 1);
            gamepadTurnDirection = Range.clip(gamepadTurnDirection, -1, 1);

            mecanumDriveSystem.mecanumWheelDrive(gamepadStrafeDirection, gamepadStrafeThrottle, gamepadTurnDirection, gamepadTurnThrottle);


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

    /*double scaleInput(double dVal) {
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

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
    }*/
}
