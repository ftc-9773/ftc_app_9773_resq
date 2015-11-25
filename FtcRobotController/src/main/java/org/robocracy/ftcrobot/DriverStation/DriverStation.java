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

    public int getMoveAngle(double x, double y){
        x = (int) x;
        y = (int) y;
        int moveAngle = 0;

        

        return(moveAngle);
    }


}
