package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.robocracy.ftcrobot.FTCRobot;

/**
 * @author Team Robocracy
 *
 * OpMode that is activated by the driver to start Teleop mode. On activation, runs {@link FTCRobot#runRobotTeleop()}.
 */
public class TeleOpBlue extends LinearOpMode {
    FTCRobot myRobot;

    @Override
    public void runOpMode() throws InterruptedException {
        // The 2nd parameter indicates whether the robot is part of the blue alliance or not.
        // This is used in the autonomous mode; it does not matter for the teleop mode,
        // but some value has to be passed, so pass the value "true".
        String writeFilePath = "/sdcard/FIRST/autonomousLog/" + System.nanoTime() + ".csv";
        String readFilePath = null;
        this.myRobot = new FTCRobot(this, readFilePath, writeFilePath, true, FTCRobot.currentlyRecording.NONE);


        waitOneFullHardwareCycle();

        waitForStart();

        myRobot.runRobotTeleop();
    }
}
