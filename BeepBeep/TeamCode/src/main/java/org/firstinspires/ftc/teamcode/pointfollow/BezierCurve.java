package org.firstinspires.ftc.teamcode.pointfollow;

import org.firstinspires.ftc.teamcode.beepbeep.BezierCurveCalc;

public class BezierCurve {
    private double a, b, c, d;

    /**
     * @param x1 - start
     * @param x2 - interp1
     * @param x3 - interp2
     * @param x4 - end
     */
    public BezierCurve(double x1, double x2, double x3, double x4) {
        this.a = x1;
        this.b = x2;
        this.c = x3;
        this.d = x4;
    }

    public double bezier_get(double u) {
        return Math.pow((1 - u), 3) * a + 3 * u * Math.pow((1 - u), 2) * b + 3 * Math.pow(u, 2) * (1 - u) * c + Math.pow(u, 3) * d;
    }

    public double bezier_deriv(double u) {
        return -3 * a * Math.pow((1 - u), 2) + 3 * b * (3 * Math.pow(u, 2) - 4 * u + 1) + 3 * c * (2 * u - 3 * Math.pow(u, 2)) + 3 * d * Math.pow(u, 2);
    }

    public double bezier_deriv2(double u) {
        return -6*a*u + 6*a + 18*b*u - 12*b - 18*c*u + 6*c + 6*d*u;
    }
}

/**
 * 1. Get curve length for x and y
 * 2. Calculate motion profile for x and y
 * 3. Get current length and velocities for x and y
 * 4. Convert length into x and y position
 * 5. Position --> PID, velocities --> FF
 */