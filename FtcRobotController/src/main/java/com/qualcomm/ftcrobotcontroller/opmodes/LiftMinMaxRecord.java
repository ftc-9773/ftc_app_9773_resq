package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.robocracy.ftcrobot.FTCRobot;

/**
 * Created by pranavb on 2/19/16.
 */
public class LiftMinMaxRecord extends LinearOpMode {
    FTCRobot robot;
    String writeFilePath = "/sdcard/FIRST/liftEncoderValues.csv";
    String readFilePath = null;

    @Override
    public void runOpMode() throws InterruptedException{
        this.robot = new FTCRobot(this, readFilePath, writeFilePath, true, FTCRobot.currentlyRecording.RECORDING_ENCODER_VALUES);

        waitOneFullHardwareCycle();

        waitForStart();

        robot.timestamp = System.nanoTime();
        robot.runRobotTeleop();
        robot.close();
    }
}
