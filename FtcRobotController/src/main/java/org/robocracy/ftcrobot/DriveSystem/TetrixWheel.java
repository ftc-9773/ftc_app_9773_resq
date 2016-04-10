package org.robocracy.ftcrobot.DriveSystem;
import org.robocracy.ftcrobot.DriveSystem.Wheel;
/**
 * Created by pranavb on 4/10/16.
 */
public class TetrixWheel extends Wheel {
    public TetrixWheel(double diameter, double frictionCoeffMatStraight){
        super(WheelType.Tetrix, diameter, frictionCoeffMatStraight, 0);
    }
}
