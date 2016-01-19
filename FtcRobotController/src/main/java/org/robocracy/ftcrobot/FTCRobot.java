package org.robocracy.ftcrobot;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.ftcrobotcontroller.opmodes.AutonomousBlue;
import com.qualcomm.ftcrobotcontroller.opmodes.AutonomousRed;
import com.qualcomm.ftcrobotcontroller.opmodes.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;

import org.robocracy.ftcrobot.DriveSystem.AWDMecanumDS;
import org.robocracy.ftcrobot.DriverStation.DriverCommand;
import org.robocracy.ftcrobot.DriverStation.DriverStation;
import org.robocracy.ftcrobot.util.FileRW;

import java.io.IOException;


/**
 * @author Team Robocracy
 * {@docRoot}
 *
 * Top level class in hierarchy. Represents an {@code FTCRobot} with
 *     main {@link FTCRobot#runRobotAutonomous()} and {@link FTCRobot#runRobotTeleop()} methods,
 *     which are used in {@link AutonomousRed}, {@link AutonomousBlue}, and {@link TeleOp} opmodes.
 */
public class FTCRobot {
    LinearOpMode curOpmode;
    public AWDMecanumDS driveSys;
    DeviceInterfaceModule dim;
    Harvester harvester;
    LinearLift linearLift;
    DcMotor harvesterMotor;
    AutonomousScorer autoScorer;
    Servo rightLatch;
    Servo leftLatch;
    Servo bucketServo;
    Servo rightClimberServo;
    Servo leftClimberServo;
    Latch latch;
    Bucket bucket;
    LeftClimber leftClimber;
    RightClimber rightClimber;
    public FileRW readFileRW, writeFileRW;
    public long timestamp;
    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;

    DriverStation drvrStation;

    public final int NAVX_DIM_I2C_PORT = 5;
    public AHRS navxDevice;

    public FTCRobot(LinearOpMode curOpmode, String readFilePath, String writeFilePath, boolean allianceIsBlue) {
        this.curOpmode = curOpmode;
        try{
            this.dim = curOpmode.hardwareMap.deviceInterfaceModule.get("dim");
            this.harvesterMotor = curOpmode.hardwareMap.dcMotor.get("harvesterMotor");
            this.navxDevice = AHRS.getInstance(curOpmode.hardwareMap.deviceInterfaceModule.get("dim"),
                    NAVX_DIM_I2C_PORT, AHRS.DeviceDataType.kProcessedData, NAVX_DEVICE_UPDATE_RATE_HZ);
            this.leftLatch = curOpmode.hardwareMap.servo.get("leftLatch");
            this.rightLatch = curOpmode.hardwareMap.servo.get("rightLatch");
            this.rightClimberServo = curOpmode.hardwareMap.servo.get("rightClimber");
            this.leftClimberServo = curOpmode.hardwareMap.servo.get("leftClimber");
        }
        catch(Exception e){
            DbgLog.error(String.format("%s . Device skipped", e.getMessage()));
        }
        this.drvrStation = new DriverStation(curOpmode, this);
        this.harvester = new Harvester(this, curOpmode, harvesterMotor);
        this.linearLift = new LinearLift(this, curOpmode);
        this.autoScorer = new AutonomousScorer(this, curOpmode, allianceIsBlue);
        this.driveSys = new AWDMecanumDS(curOpmode, this);
        this.timestamp = System.nanoTime();
        this.bucketServo = curOpmode.hardwareMap.servo.get("bucketServo");
        this.latch = new Latch(this, leftLatch, rightLatch, curOpmode);
        this.bucket = new Bucket(this, curOpmode, bucketServo);
        this.leftClimber = new LeftClimber(this, leftClimberServo, curOpmode);
        this.rightClimber = new RightClimber(this, rightClimberServo, curOpmode);

        if (readFilePath != null) {
            this.readFileRW = new FileRW(readFilePath, false);
        } else {
            this.readFileRW = null;
        }
        if (writeFilePath != null) {
            this.writeFileRW = new FileRW(writeFilePath, true);
        } else {
            this.writeFileRW = null;
        }
    }

    /**
     * Runs Autonomous mode with values from file in {@code filePath}.
     * @throws InterruptedException
     */
    public void runRobotAutonomous()  throws InterruptedException {

        autoScorer.driveUsingReplay();
//        this.autoScorer.step1_driveToRepairZone(this.driveSys);
        //this.autoScorer.step2_alignWithWhiteLine(this.driveSys);
        //this.autoScorer.step3_moveToTheRescueBeacon(this.driveSys);
        //this.autoScorer.step4_moveBackToMountainBase(this.driveSys);
/*
        this.driveSys.autoMecanum(250, 82, 12, 0);
        this.driveSys.autoMecanum(0, 0, 12, 70);
*/
        try {
            if (readFileRW != null){
                readFileRW.close();
            }
            if (writeFileRW != null) {
                writeFileRW.close();
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void runRobotAutonomous(int distanceToStrafe) throws InterruptedException {
        this.autoScorer.strafeTheDistance(driveSys, distanceToStrafe);
    }

    /**
     * Runs TeleOp mode by {@link DriverStation#getNextCommand()} for getting gamepad values.
     * @throws InterruptedException
     */
    public  void  runRobotTeleop() throws InterruptedException {
        DriverCommand driverCommand;
        while(curOpmode.opModeIsActive()){
            driverCommand = drvrStation.getNextCommand();
            this.driveSys.applyCmd(driverCommand);

            this.harvester.applyDSCmd(driverCommand);
            this.linearLift.applyCmd(driverCommand);

            this.latch.applyDSCmd(driverCommand);
            this.bucket.applyDSCmd(driverCommand);
            this.leftClimber.applyDSCmd(driverCommand);
            this.rightClimber.applyDSCmd(driverCommand);

            // Wait for one hardware cycle for the setPower(0) to take effect.
            this.curOpmode.waitForNextHardwareCycle();

        }
        try {
            if (readFileRW != null){
                readFileRW.close();
            }
            if (writeFileRW != null) {
                writeFileRW.close();
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
