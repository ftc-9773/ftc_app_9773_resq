package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.robocracy.ftcrobot.FTCRobot;
import org.robocracy.ftcrobot.util.FileRW;

import java.io.IOException;
import java.util.concurrent.TimeUnit;

/**
 * @author Team Robocracy
 *
 * OpMode that is activated by driver. On activation, runs {@link FTCRobot#runRobotAutonomous()},
 *   passing {@code filePath} as the path to the Blue Alliance autonomous instruction file.
 */
public class AutonomousBlue extends LinearOpMode {
    FTCRobot robot;
    final int EXPECTED_DISTANCE=72; // inches
    int delayInSeconds=0;
    int distanceFromWall=EXPECTED_DISTANCE; // inches from the parimeter wall on the left side
                                            // to the left edge of the robot

    @Override
    public void runOpMode() throws InterruptedException{
        String readFilePath = "/sdcard/FIRST/autonomousCmds/blue.csv";
        String writeFilePath = "/sdcard/FIRST/autonomousLog/" + System.nanoTime() + ".csv";
        String autonomousConfigFile = "/sdcard/FIRST/autonomousCmds/config.txt";

        this.robot = new FTCRobot(this, readFilePath, writeFilePath, true);

        waitOneFullHardwareCycle();

        waitForStart();

        getAutonomousParameters(autonomousConfigFile);
        if (delayInSeconds > 0){
            TimeUnit.SECONDS.sleep(delayInSeconds);
        }
        if (distanceFromWall != EXPECTED_DISTANCE) {
            this.robot.runRobotAutonomous((distanceFromWall - EXPECTED_DISTANCE));
        }

        robot.runRobotAutonomous();
    }

    void getAutonomousParameters(String autonomousConfigFilePath) {
        FileRW fileRW;
        String line;
        String[] lineArray;

        if (autonomousConfigFilePath != null) {
            fileRW = new FileRW(autonomousConfigFilePath, false);
            line = fileRW.getNextLine();
            while (line != null) {
                lineArray = line.split("=");
                if (lineArray[0].matches("startingDelay")) {
                    delayInSeconds = Integer.parseInt(lineArray[1]);
                }
                if (lineArray[0].matches("distanceFromWall")) {
                    distanceFromWall = Integer.parseInt(lineArray[1]);
                }
                line = fileRW.getNextLine();
            }
            try {
                fileRW.close();
            }
            catch (IOException e) {
                e.printStackTrace();
            }
        }
    }
}
