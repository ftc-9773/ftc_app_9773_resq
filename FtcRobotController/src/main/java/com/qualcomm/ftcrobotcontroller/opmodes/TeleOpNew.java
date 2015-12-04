package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.robocracy.ftcrobot.FTCRobot;

/**
 * Created by Robocracy on 11/19/2015.
 */
public class TeleOpNew extends LinearOpMode {
    FTCRobot myRobot;

    @Override
    public void runOpMode() throws InterruptedException {
        // The 2nd parameter indicates whether the robot is part of the blue alliance or not.
        // This is used in the autonomous mode; it does not matter for the teleop mode,
        // but some value has to be passed, so pass the value "true".
        this.myRobot = new FTCRobot(this, true);


        waitOneFullHardwareCycle();

        waitForStart();

        myRobot.runRobotTeleop();
    }
}
