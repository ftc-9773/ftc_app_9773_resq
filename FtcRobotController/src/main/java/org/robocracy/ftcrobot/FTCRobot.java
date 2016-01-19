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
    DeviceInterfaceModule dim = null;
    Harvester harvester = null;
    LinearLift linearLift = null;
    DcMotor harvesterMotor = null;
    AutonomousScorer autoScorer;
    Servo rightLatch = null;
    Servo leftLatch = null;
    Servo bucketServo = null;
    Servo rightClimberServo = null;
    Servo leftClimberServo = null;
    Latch latch = null;
    Bucket bucket = null;
    LeftClimber leftClimber = null;
    RightClimber rightClimber = null;
    public FileRW readFileRW, writeFileRW;
    public long timestamp;
    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;

    DriverStation drvrStation;

    public final int NAVX_DIM_I2C_PORT = 5;
    public AHRS navxDevice = null;

    public FTCRobot(LinearOpMode curOpmode, String readFilePath, String writeFilePath, boolean allianceIsBlue) {
        this.curOpmode = curOpmode;
        initDevice("dim");
        initDevice("navx");
        initDevice("harvesterMotor");
        initDevice("leftLatch");
        initDevice("rightLatch");
        initDevice("leftClimber");
        initDevice("rightClimber");
        initDevice("bucketServo");

/*
        this.dim = curOpmode.hardwareMap.deviceInterfaceModule.get("dim");
        this.harvesterMotor = curOpmode.hardwareMap.dcMotor.get("harvesterMotor");
        this.navxDevice = AHRS.getInstance(curOpmode.hardwareMap.deviceInterfaceModule.get("dim"),
                NAVX_DIM_I2C_PORT, AHRS.DeviceDataType.kProcessedData, NAVX_DEVICE_UPDATE_RATE_HZ);
        this.leftLatch = curOpmode.hardwareMap.servo.get("leftLatch");
        this.rightLatch = curOpmode.hardwareMap.servo.get("rightLatch");
        this.rightClimberServo = curOpmode.hardwareMap.servo.get("rightClimber");
        this.leftClimberServo = curOpmode.hardwareMap.servo.get("leftClimber");
        this.bucketServo = curOpmode.hardwareMap.servo.get("bucketServo");
*/

        this.drvrStation = new DriverStation(curOpmode, this);
        this.harvester = new Harvester(this, curOpmode, harvesterMotor);
        this.linearLift = new LinearLift(this, curOpmode);
        this.autoScorer = new AutonomousScorer(this, curOpmode, allianceIsBlue);
        this.driveSys = new AWDMecanumDS(curOpmode, this);
        this.timestamp = System.nanoTime();
        this.latch = new Latch(this, leftLatch, rightLatch, curOpmode);
        this.bucket = new Bucket(this, curOpmode, bucketServo);
        this.leftClimber = new LeftClimber(this, leftClimberServo, curOpmode);
        this.rightClimber = new RightClimber(this, rightClimberServo, curOpmode);

        if (readFilePath != null) {
            this.readFileRW = new FileRW(readFilePath, false);
            DbgLog.msg(String.format("readFilePath is %s", readFilePath));
        } else {
            this.readFileRW = null;
            DbgLog.msg(String.format("readFilePath is null"));
        }
        if (writeFilePath != null) {
            this.writeFileRW = new FileRW(writeFilePath, true);
            DbgLog.msg(String.format("writeFilePath is %s", writeFilePath));
        } else {
            this.writeFileRW = null;
            DbgLog.msg(String.format("writeFilePath is null"));
        }
    }

    void initDevice (String deviceName) {
        try {
            if (deviceName.matches("dim")) {
                this.dim = curOpmode.hardwareMap.deviceInterfaceModule.get("dim");
            } else if (deviceName.matches("navx") && (this.dim != null)) {
                this.navxDevice = AHRS.getInstance(this.dim,
                        NAVX_DIM_I2C_PORT, AHRS.DeviceDataType.kProcessedData, NAVX_DEVICE_UPDATE_RATE_HZ);
            } else if (deviceName.matches("harvesterMotor")) {
                this.harvesterMotor = curOpmode.hardwareMap.dcMotor.get("harvesterMotor");
            } else if (deviceName.matches("leftLatch")) {
                this.leftLatch = curOpmode.hardwareMap.servo.get("leftLatch");
            } else if (deviceName.matches("rightLatch")) {
                this.rightLatch = curOpmode.hardwareMap.servo.get("rightLatch");
            } else if (deviceName.matches("leftClimber")) {
                this.leftClimberServo = curOpmode.hardwareMap.servo.get("leftClimber");
            } else if (deviceName.matches("rightClimber")) {
                this.rightClimberServo = curOpmode.hardwareMap.servo.get("rightClimber");
            } else if (deviceName.matches("bucketServo")) {
                this.bucketServo = curOpmode.hardwareMap.servo.get("bucketServo");
            }
        }
        catch(Exception e){
            DbgLog.error(String.format("%s . Device skipped", e.getMessage()));
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
/*
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
*/
    }
}
