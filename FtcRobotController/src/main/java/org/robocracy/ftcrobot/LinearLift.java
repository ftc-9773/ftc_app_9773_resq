package org.robocracy.ftcrobot;

import org.robocracy.ftcrobot.FTCRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.robocracy.ftcrobot.DriverStation.DriverCommand;
/**
 * Created by pranavb on 11/29/15.
 */
public class LinearLift {
    FTCRobot robot;
    LinearOpMode curOpMode;
    DcMotor liftAngleMotor;
    DcMotor liftDirectionMotor;

    public LinearLift(FTCRobot robot, LinearOpMode curOpMode){
        this.robot = robot;
        this.curOpMode = curOpMode;
        this.liftAngleMotor = curOpMode.hardwareMap.dcMotor.get("liftAngleMotor");
        this.liftDirectionMotor = curOpMode.hardwareMap.dcMotor.get("liftDirectionMotor");
    }

    public void applyCmd(DriverCommand driverCommand){
        liftDirectionMotor.setPower(driverCommand.linliftcmd.direction);
        liftAngleMotor.setPower(driverCommand.linliftcmd.angle);
    }
}
