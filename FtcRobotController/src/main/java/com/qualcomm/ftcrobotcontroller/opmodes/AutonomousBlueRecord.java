package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.robocracy.ftcrobot.FTCRobot;
import org.robocracy.ftcrobot.util.FileRW;

/**
 * @author Team Robocracy
 *
 * OpMode that is activated by driver. On activation, runs {@link FTCRobot#runRobotAutonomous(String)}, passing {@code filePath}
 * as the path to the Blue Alliance autonomous instruction file.
 */
public class AutonomousBlueRecord extends LinearOpMode {
    FTCRobot robot;
    String filePath = "/sdcard/FIRST/autonomousCmds/blue.csv";

    @Override
    public void runOpMode() throws InterruptedException{
        this.robot = new FTCRobot(this, true);
        this.robot.driveSys.setFileHandle(filePath, true);

        waitOneFullHardwareCycle();

        waitForStart();

        robot.runRobotTeleop();
    }
}
