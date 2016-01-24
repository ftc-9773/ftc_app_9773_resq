package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.robocracy.ftcrobot.FTCRobot;

/**
 * Created by pranavb on 1/24/16.
 */
public class EndGameRedRecord extends LinearOpMode {
    FTCRobot robot;
    String writeFilePath = "/sdcard/FIRST/autonomousCmds/endGameRed.csv";
    String readFilePath = null;

    @Override
    public void runOpMode() throws InterruptedException{
        this.robot = new FTCRobot(this, readFilePath, writeFilePath, false, FTCRobot.currentlyRecording.RECORDING_ENDGAME);

        waitOneFullHardwareCycle();

        waitForStart();

        robot.runRobotTeleop();
    }
}
