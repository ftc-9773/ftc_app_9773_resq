package com.qualcomm.ftcrobotcontroller.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.robocracy.ftcrobot.FTCRobot;
import org.robocracy.ftcrobot.util.NavX;

/**
 * Created by pranavb on 12/24/15.
 */
public class NavXTest extends LinearOpMode{
    FTCRobot robot;

    @Override
    public void runOpMode() throws InterruptedException{
        this.robot = new FTCRobot(this, null, null, true, FTCRobot.currentlyRecording.NONE);

        waitOneFullHardwareCycle();

        while (opModeIsActive()){

        }
    }
}
