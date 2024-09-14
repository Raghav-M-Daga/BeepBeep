package org.firstinspires.ftc.teamcode.beepbeep;

import org.firstinspires.ftc.teamcode.pointfollow.BezierCurve;

public class BezierCurveCalc {

    private double[] sums, upsilon;

    public double[] get_sums() {
        return this.sums;
    }
    public double[] get_upsilon() {
        return this.upsilon;
    }

    public double bezier_param_of_disp(double s, double[] sums, double[] upsilon) {
        for (int i = 0; i < sums.length; i++) {
            if (s <= sums[i]) {
                return i / 100.0;
            }
        }

        return 0.0;
    }

    public double bezier_param_of_disp_deriv(BezierCurve x_bezier, BezierCurve y_bezier, double u) {
        double sum = Math.pow(x_bezier.bezier_deriv(u), 2) + Math.pow(y_bezier.bezier_deriv(u), 2);
        return 1.0 / (Math.sqrt(sum));
    }

    public double bezier_param_of_disp_deriv2(BezierCurve x_bezier, BezierCurve y_bezier, double u) {
        double dxdu = x_bezier.bezier_deriv(u);
        double dydu = y_bezier.bezier_deriv(u);
        double dx2du = x_bezier.bezier_deriv2(u);
        double dy2du = y_bezier.bezier_deriv2(u);

        return -1 * (dxdu*dx2du + dydu*dy2du) / (Math.pow(Math.pow(dxdu, 2) + Math.pow(dydu, 2), 2));
    }

    public double bezier_length(BezierCurve x_bezier, BezierCurve y_bezier) {
        double[] upsilon = new double[100];
        double[] integrand = new double[100];
        double[] sums = new double[100];
        for (int i = 0; i < 100; i += 1) {
            upsilon[i] = i / 100;
        }
        double dupsilon = 0.01;
        double last_sum = 0;

        for (int i = 0; i < 100; i++) {
            integrand[i] = Math.sqrt(Math.pow(x_bezier.bezier_deriv(i/100), 2) + Math.pow(y_bezier.bezier_deriv(i/100), 2));
            sums[i] = last_sum + integrand[i] * dupsilon;
            last_sum = sums[i];
        }
        this.sums = sums;
        this.upsilon = upsilon;

        return sums[99];
    }
}
