package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
/*import com.qualcomm.robotcore.eventloop.opmode.OpMode;*/
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.ftcrobotcontroller.opmodes.DriveSystem;
import com.qualcomm.robotcore.hardware.DcMotorController;
import org.robocracy.ftcrobot.FTCRobot;

/**
 * Created by pranavb on 11/11/15.
 */
public class AutonomousBlue extends LinearOpMode {
    FTCRobot myRobot;

    @Override
    public void runOpMode() throws InterruptedException {
        this.myRobot = new FTCRobot(this, true);


        waitOneFullHardwareCycle();

        waitForStart();

        myRobot.runRobotAutonomous();

    }
}
