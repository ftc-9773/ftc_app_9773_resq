package org.robocracy.ftcrobot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.robocracy.ftcrobot.DriveSystem.AWDMecanumDS;
import org.robocracy.ftcrobot.DriveSystem.DriveSystemInterface;


/**
 * Created by Robocracy on 11/17/2015.
 */
public class FTCRobot {
    LinearOpMode curOpmode;
    AWDMecanumDS driveSys;
    OpticalDistanceSensor opd;
    DeviceInterfaceModule dim;

    public FTCRobot(LinearOpMode curOpmode) {
        this.driveSys = new AWDMecanumDS(curOpmode);
        this.curOpmode = curOpmode;
        this.dim = curOpmode.hardwareMap.deviceInterfaceModule.get("dim");
        this.opd = curOpmode.hardwareMap.opticalDistanceSensor.get("opd");
    }

    public void runRobotAutonomous()  throws InterruptedException {


        this.driveSys.autoMove(DriveSystemInterface.RobotDirection.FORWARD, 60.0, 3);
//        curOpmode.waitOneFullHardwareCycle();

//        double counts = mecanumDriveSystem.mecanumWheelAutoDrive(60, 0.5);

//        curOpmode.waitOneFullHardwareCycle();
        /*mecanumDriveSystem.mecanumWheelDrive(0, 0, 0, 1);
        sleep(2000);
        mecanumDriveSystem.mecanumWheelDrive(0, 0, 0, 0);*/

/*
        while(curOpmode.opModeIsActive()){

            curOpmode.telemetry.addData("Counts to move: ", counts);
            curOpmode.telemetry.addData("front left counts: ", frontLeft.getCurrentPosition());
            curOpmode.telemetry.addData("front right counts: ", frontRight.getCurrentPosition());
            curOpmode.telemetry.addData("rear left counts: ", rearLeft.getCurrentPosition());
            curOpmode.telemetry.addData("rear right counts: ", rearRight.getCurrentPosition());
        }
*/

    }

    public  void  runRobotTeleop() throws InterruptedException {
        this.driveSys.driverMove(DriveSystemInterface.RobotDirection.FORWARD, 1);
    }
}
