package org.firstinspires.ftc.teamcode.pointfollow;

import com.acmerobotics.roadrunner.geometry.Vector2d;

public class CurvePointCalc {

    private double[] sums, upsilon;
    private double[] x_points, y_points, u_points;

    BezierCurve x_bezier, y_bezier;

    public CurvePointCalc(BezierCurve x_bezier, BezierCurve y_bezier) {
        this.x_bezier = x_bezier;
        this.y_bezier = y_bezier;

        bezier_points_at_intervals_in_inches(3);
    }

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

    public double bezier_param_of_disp_deriv(double u) {
        double sum = Math.pow(x_bezier.bezier_deriv(u), 2) + Math.pow(y_bezier.bezier_deriv(u), 2);
        return 1.0 / (Math.sqrt(sum));
    }

    public double bezier_param_of_disp_deriv2(double u) {
        double dxdu = x_bezier.bezier_deriv(u);
        double dydu = y_bezier.bezier_deriv(u);
        double dx2du = x_bezier.bezier_deriv2(u);
        double dy2du = y_bezier.bezier_deriv2(u);

        return -1 * (dxdu*dx2du + dydu*dy2du) / (Math.pow(Math.pow(dxdu, 2) + Math.pow(dydu, 2), 2));
    }

    public double bezier_length() {
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

    public void bezier_points_at_intervals_in_inches(double interval_inches) {
        double curve_length = bezier_length();
        int num_points = (int) (curve_length / interval_inches);

        if (num_points == 0) num_points = 1;

        double interval_u = 1 / num_points;
        double[] u_values = new double[num_points + 1];

        for (int i = 0; i < u_values.length; i++) u_values[i] = i * 1.0 / num_points;

        this.x_points = new double[u_values.length];
        this.y_points = new double[u_values.length];

        for (int i = 0; i < u_values.length; i++) {
            this.x_points[i] = x_bezier.bezier_get(u_values[i]);
            this.y_points[i] = y_bezier.bezier_get(u_values[i]);
        }

        this.u_points = u_values;
    }

    public double[] getX_points() {
        return x_points;
    }

    public double[] getY_points() {
        return y_points;
    }

    public double[] getU_points() {
        return u_points;
    }

    public int getIndex(double x) {
        for (int i = 0; i < x_points.length; i++) {
            if (x == x_points[i]) {
                return i;
            }
        }

        return 0;
    }

    public double angle_tangent(Vector2d point) {
        double u = this.u_points[getIndex(point.getX())];

        double dy_dx = x_bezier.bezier_deriv(u) / y_bezier.bezier_deriv(u);
        double angle = Math.atan(dy_dx);

        return angle;
    }

    public Vector2d closest_point(Vector2d currPose) {
        double minDist = Math.sqrt(Math.pow(currPose.getX() - this.x_points[0], 2) + Math.pow(currPose.getY() - this.y_points[0], 2));
        Vector2d minPoint = new Vector2d(x_points[0], y_points[0]);

        for (int i = 0; i < this.x_points.length - 1; i++) {
//            double x = this.x_points[i];
//            double y = this.y_points[i];
            double dist = Math.sqrt(Math.pow(currPose.getX() - this.x_points[i], 2) + Math.pow(currPose.getY() - this.y_points[i], 2));

            if (dist < minDist) {
                minPoint = new Vector2d(x_points[i+1], y_points[i+1]);
                minDist = dist;
            }
        }

        return minPoint;
    }
}
