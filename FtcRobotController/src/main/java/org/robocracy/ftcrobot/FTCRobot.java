package org.robocracy.ftcrobot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;

import org.robocracy.ftcrobot.DriveSystem.AWDMecanumDS;
import org.robocracy.ftcrobot.DriverStation.DriverCommand;
import org.robocracy.ftcrobot.DriverStation.DriverStation;


/**
 * Created by Robocracy on 11/17/2015.
 */
public class FTCRobot {
    LinearOpMode curOpmode;
    AWDMecanumDS driveSys;
    DeviceInterfaceModule dim;
    Harvester harvester;
    LinearLift linearLift;
    DcMotor harvesterMotor;
    AutonomousScorer autoScorer;
    // RobotLength = Distance in inches from the center of front left to the center of rear left wheel
    double RobotLength;
    // RobotWidth = Distance in inches from the center of front left to the center of front right wheel
    double RobotWidth;

    DriverStation drvrStation;

    public FTCRobot(LinearOpMode curOpmode, boolean allianceIsBlue) {
        this.driveSys = new AWDMecanumDS(curOpmode, this);
        this.curOpmode = curOpmode;
        this.dim = curOpmode.hardwareMap.deviceInterfaceModule.get("dim");
        this.drvrStation = new DriverStation(curOpmode, this);
        this.harvesterMotor = curOpmode.hardwareMap.dcMotor.get("harvesterMotor");
        this.harvester = new Harvester(this, curOpmode, harvesterMotor);
        this.linearLift = new LinearLift(this, curOpmode);
        this.autoScorer = new AutonomousScorer(this, curOpmode, allianceIsBlue);
    }

    public void runRobotAutonomous()  throws InterruptedException {

        this.autoScorer.step1_driveToRepairZone(this.driveSys);
        this.autoScorer.step2_alignWithWhiteLine(this.driveSys);
        this.autoScorer.step3_moveToTheRescueBeacon(this.driveSys);
        this.autoScorer.step4_moveBackToMountainBase(this.driveSys);
/*
        this.driveSys.autoMecanum(250, 82, 12, 0);
        this.driveSys.autoMecanum(0, 0, 12, 70);
*/
    }

    public  void  runRobotTeleop() throws InterruptedException {
        DriverCommand driverCommand;
        while(curOpmode.opModeIsActive()){
            driverCommand = drvrStation.getNextCommand();
            //DbgLog.msg(String.format("angle = %f, speedMult= %f, Omega = %f",
            //        driverCommand.drvsyscmd.angle, driverCommand.drvsyscmd.speedMultiplier, driverCommand.drvsyscmd.Omega));
            this.driveSys.applyCmd(driverCommand);
//            this.driveSys.driveMecanum((int) driverCommand.drvsyscmd.angle, driverCommand.drvsyscmd.speedMultiplier, driverCommand.drvsyscmd.Omega);

            this.harvester.applyDSCmd(driverCommand);
            this.linearLift.applyCmd(driverCommand);
            // Wait for one hardware cycle for the setPower(0) to take effect.
            this.curOpmode.waitForNextHardwareCycle();
        }
    }
}
