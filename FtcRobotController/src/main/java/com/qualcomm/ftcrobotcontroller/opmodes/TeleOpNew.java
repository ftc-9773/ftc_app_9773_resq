package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.robocracy.ftcrobot.FTCRobot;

/**
 * @author Team Robocracy
 *
 * OpMode that is activated by the driver to start TeleOp mode. On activation, runs {@link FTCRobot#runRobotTeleop()}.
 */
public class TeleOpNew extends LinearOpMode {
    FTCRobot myRobot;

    @Override
    public void runOpMode() throws InterruptedException {
        // The 2nd parameter indicates whether the robot is part of the blue alliance or not.
        // This is used in the autonomous mode; it does not matter for the teleop mode,
        // but some value has to be passed, so pass the value "true".
        this.myRobot = new FTCRobot(this, true);
        String filePath = "/sdcard/FIRST/autonomousLog/" + System.nanoTime() + ".csv";


        waitOneFullHardwareCycle();

        waitForStart();

        this.myRobot.driveSys.setFileHandle(filePath, true);

        myRobot.runRobotTeleop();
    }
}
