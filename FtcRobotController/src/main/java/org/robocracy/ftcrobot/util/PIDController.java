package org.robocracy.ftcrobot.util;


/**
 * Created by Robocracy on 11/17/2015.
 */
public class PIDController {
    double Kp, Ki,Kd;
    int ArrayLenForIntegral;
    double setPoint, errorMax;

    public PIDController(double Kp, double Ki, double Kd, int ArrayLen,
                         double setPoint, double errorMax) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.ArrayLenForIntegral = ArrayLen;
        this.setPoint = setPoint;
        this.errorMax = errorMax;
    }

    public double getNextOutputValue(double curValue) {
        double correction = 0.0;
        // Calculate error
        double error = setPoint - curValue;
        // Save error to queue
        // Calculate Correction
        // Return the correction

        return (correction);
    }

    public void setSetPoint(double setPoint) {
        this.setPoint = setPoint;
    }

    public void reset(){
        this.Kp = 0.0;
        this.Ki = 0.0;
        this.Kd = 0.0;
        this.setPoint = 0.0;
        this.errorMax = 0.0;
    }
}
