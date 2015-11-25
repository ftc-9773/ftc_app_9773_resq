package org.robocracy.ftcrobot.DriverStation;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Robocracy on 11/19/2015.
 */
public class DriverStation {
    public static  DriverCommand drvrCmd = new DriverCommand();

    public DriverCommand getNextCommand(){
        return (drvrCmd);
    }
}
