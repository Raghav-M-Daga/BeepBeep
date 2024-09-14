package org.firstinspires.ftc.teamcode.beepbeep;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    public double kP;
    public double kI;
    public double kD;

    ElapsedTime clock = new ElapsedTime();

    double currentError = 0.0;
    double previousError = 0.0;

    private static double integralSum = 0.0;
    private static double maxIntegralSum = integralSum;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double calculate(double target, double current) {
        currentError = target - current;

        double p = kP * currentError;

        integralSum = integralSum + (currentError * clock.seconds());
        double i = integralSum * kI;

        double d = kD * (currentError - previousError) / clock.seconds();
        previousError = currentError;

        maxIntegralSum = integralSum;
        clock.reset();

        return (p + i + d);
    }
}
