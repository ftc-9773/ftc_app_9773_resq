package org.robocracy.ftcrobot;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.robocracy.ftcrobot.DriveSystem.AWDMecanumDS;
import org.robocracy.ftcrobot.DriveSystem.DriveSystemInterface;
import org.robocracy.ftcrobot.DriverStation.DriverCommand;
import org.robocracy.ftcrobot.DriverStation.DriverStation;


/**
 * Created by Robocracy on 11/17/2015.
 */
public class FTCRobot {
    LinearOpMode curOpmode;
    AWDMecanumDS driveSys;
    public OpticalDistanceSensor opd;
    DeviceInterfaceModule dim;
    Harvester harvester;
    LinearLift linearLift;
    DcMotor harvesterMotor;
    DcMotor liftAngleMotor;
    DcMotor liftDirectionMotor;
    // RobotLength = Distance in inches from the center of front left to the center of rear left wheel
    double RobotLength;
    // RobotWidth = Distance in inches from the center of front left to the center of front right wheel
    double RobotWidth;

    DriverStation drvrStation;

    public FTCRobot(LinearOpMode curOpmode) {
        this.driveSys = new AWDMecanumDS(curOpmode, this);
        this.curOpmode = curOpmode;
        this.dim = curOpmode.hardwareMap.deviceInterfaceModule.get("dim");
        this.opd = curOpmode.hardwareMap.opticalDistanceSensor.get("opd");
        this.drvrStation = new DriverStation(curOpmode, this);
        this.harvesterMotor = curOpmode.hardwareMap.dcMotor.get("harvesterMotor");
        this.harvester = new Harvester(this, curOpmode, harvesterMotor);
        this.liftAngleMotor = curOpmode.hardwareMap.dcMotor.get("liftAngleMotor");
        this.liftDirectionMotor = curOpmode.hardwareMap.dcMotor.get("liftDirectionMotor");
        this.linearLift = new LinearLift(this, curOpmode, liftAngleMotor, liftDirectionMotor);
    }

    public void runRobotAutonomous()  throws InterruptedException {


        this.driveSys.autoMecanum(230, 82, 12, 0);
        //this.driveSys.autoMove(DriveSystemInterface.RobotDirection.BACKWARD, 60.0, 12);
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
        DriverCommand driverCommand;
        while(curOpmode.opModeIsActive()){
            driverCommand = drvrStation.getNextCommand();
            //DbgLog.msg(String.format("angle = %f, speedMult= %f, Omega = %f",
            //        driverCommand.drvsyscmd.angle, driverCommand.drvsyscmd.speedMultiplier, driverCommand.drvsyscmd.Omega));
            this.driveSys.driveMecanum((int) driverCommand.drvsyscmd.angle, driverCommand.drvsyscmd.speedMultiplier, driverCommand.drvsyscmd.Omega);

            this.harvester.applyDSCmd(driverCommand);
            this.linearLift.applyCmd(driverCommand);
            // Wait for one hardware cycle for the setPower(0) to take effect.
            this.curOpmode.waitForNextHardwareCycle();
        }
    }
}
