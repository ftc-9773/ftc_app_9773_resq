package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.robocracy.ftcrobot.FTCRobot;

/**
 * Created by pranavb on 1/24/16.
 */
public class EndGameBlueRecord extends LinearOpMode {
    FTCRobot robot;
    String writeFilePath = "/sdcard/FIRST/autonomousCmds/endGameBlue.csv";
    String readFilePath = null;
    boolean startRecording = false;

    @Override
    public void runOpMode() throws InterruptedException{
        this.robot = new FTCRobot(this, readFilePath, writeFilePath, true, FTCRobot.currentlyRecording.RECORDING_ENDGAME);

        waitOneFullHardwareCycle();

        waitForStart();

        if(gamepad1.back){
            startRecording = true;
        }
        if (startRecording) {
            robot.timestamp = System.nanoTime();
            robot.runRobotTeleop();
        }
    }
}
