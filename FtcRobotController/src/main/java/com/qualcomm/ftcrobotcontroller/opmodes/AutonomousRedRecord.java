package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.robocracy.ftcrobot.FTCRobot;
import org.robocracy.ftcrobot.util.FileRW;

/**
 * @author Team Robocracy
 *
 * OpMode that is activated by driver. On activation, runs {@link FTCRobot#runRobotAutonomous()}, passing {@code filePath}
 * as the path to the Blue Alliance autonomous instruction file.
 */
public class AutonomousRedRecord extends LinearOpMode {
    FTCRobot robot;
    String writeFilePath = "/sdcard/FIRST/autonomousCmds/red.csv";
    String readFilePath = null;

    @Override
    public void runOpMode() throws InterruptedException{
        this.robot = new FTCRobot(this, readFilePath, writeFilePath, true, FTCRobot.currentlyRecording.RECORDING_AUTONOMOUS);

        waitOneFullHardwareCycle();

        waitForStart();

        robot.runRobotTeleop();
    }
}
