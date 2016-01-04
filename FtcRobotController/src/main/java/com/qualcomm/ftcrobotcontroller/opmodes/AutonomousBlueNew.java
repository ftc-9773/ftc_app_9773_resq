package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.robocracy.ftcrobot.FTCRobot;

/**
 * @author Team Robocracy
 *
 * OpMode that is activated by driver. On activation, runs {@link FTCRobot#runRobotAutonomous(String)}, passing {@code filePath}
 * as the path to the Blue Alliance autonomous instruction file.
 */
public class AutonomousBlueNew extends LinearOpMode {
    FTCRobot robot;

    @Override
    public void runOpMode() throws InterruptedException{
        this.robot = new FTCRobot(this, true);

        waitOneFullHardwareCycle();

        waitForStart();

        robot.runRobotAutonomous("/sdcard/FIRST/autonomousCmds/blue.csv");
    }
}
