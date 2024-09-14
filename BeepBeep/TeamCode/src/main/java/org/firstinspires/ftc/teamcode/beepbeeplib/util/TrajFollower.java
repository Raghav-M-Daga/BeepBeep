package org.firstinspires.ftc.teamcode.beepbeeplib.util;

// Bezier path1 = new Bezier();
// drive.followPath(path1)
// trajFollower.followBezier(Bezier)

import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.kS_x;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.kS_y;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.kV_x;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.kV_y;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.maxAccel;
import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.maxVel;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pointfollow.BezierCurve;
import org.firstinspires.ftc.teamcode.beepbeep.BezierCurveCalc;
import org.firstinspires.ftc.teamcode.beepbeep.MotionProfile;
import org.firstinspires.ftc.teamcode.beepbeeplib.Drive;

public class TrajFollower {
    Drive dt;


    public TrajFollower(Drive dt) {
        this.dt = dt;
    }

    public void followBezier(double error, BezierCurve bezier_x, BezierCurve bezier_y, PIDController px, PIDController py, PIDController pheading, double desired_heading) {
        ElapsedTime timer = new ElapsedTime();
        timer.time();

        double control_signal_x, control_signal_y, control_signal_heading;
        double instantTargetPosition, u;
        double instantTargetVelocity, duds, dxdu, dydu, x_vel, y_vel;
        double instantTargetAcceleration, du2ds, dx2du, dy2du, x_accel, y_accel;
        double totalY, totalX;

        BezierCurveCalc bezier_calc = new BezierCurveCalc();
        double curve_length = bezier_calc.bezier_length(bezier_x, bezier_y);

        MotionProfile motionProfile = new MotionProfile(maxAccel, maxVel, curve_length);

//        while(timer.time() >= motionProfile.getTotalTime() && )
        Pose2d poseEstimate = dt.getPoseEstimate();

        while(calcError(error, poseEstimate, poseEstimate) || motionProfile.getTotalTime() >= timer.time()) {
            poseEstimate = dt.getPoseEstimate();

            // Position
            instantTargetPosition = motionProfile.getPosition(timer.time());
            u = bezier_calc.bezier_param_of_disp(instantTargetPosition, bezier_calc.get_sums(), bezier_calc.get_upsilon());
            control_signal_x = px.calculate(bezier_x.bezier_get(u), poseEstimate.getX());
            control_signal_y = py.calculate(bezier_y.bezier_get(u), poseEstimate.getY());
            control_signal_heading = pheading.calculate(angleWrap(Math.toRadians(desired_heading)), angleWrap(poseEstimate.getHeading()));

            // Velocity
            // s′(t)u′(s(t))x′(u(s(t)))
            instantTargetVelocity = motionProfile.getVelocity(timer.time());
            duds = bezier_calc.bezier_param_of_disp_deriv(bezier_x, bezier_y, u);
            dxdu = bezier_x.bezier_deriv(u);
            dydu = bezier_y.bezier_deriv(u);
            x_vel = dxdu * duds * instantTargetVelocity;
            y_vel = dydu * duds * instantTargetVelocity;

            // Acceleration
            // (s'(t))^2 * (u'(s(t)))^2 * x''(u(s(t))) + ((s'(t))^2 * u''(s(t)) + s''(t) * u'(s(t))) * x'(u(s(t)))
            instantTargetAcceleration = motionProfile.getAcceleration(timer.time());
            du2ds = bezier_calc.bezier_param_of_disp_deriv2(bezier_x, bezier_y, u);
            dx2du = bezier_x.bezier_deriv2(u);
            dy2du = bezier_y.bezier_deriv2(u);
            x_accel = Math.pow(instantTargetVelocity, 2) * Math.pow(duds, 2) * dx2du + Math.pow(instantTargetVelocity, 2) * du2ds + instantTargetAcceleration * duds * dxdu;
            y_accel = Math.pow(instantTargetVelocity, 2) * Math.pow(duds, 2) * dy2du + Math.pow(instantTargetVelocity, 2) * du2ds + instantTargetAcceleration * duds * dydu;

            // Combined
            totalX = control_signal_x + x_vel * kV_x + x_accel * kS_x;
            totalY = control_signal_y + y_vel * kV_y + y_accel * kS_y;

            Vector2d input = new Vector2d(
                    totalX,
                    totalY
            ).rotated(-poseEstimate.getHeading());

            dt.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            control_signal_heading
                    )
            );

            dt.update();
        }
    }

    private boolean calcError(double err, Pose2d curPose, Pose2d desiredPose) {
        double x = Math.pow(desiredPose.getX() - curPose.getX(), 2);
        double y = Math.pow(desiredPose.getY() - curPose.getY(), 2);
        double dist = Math.sqrt(x+y);

        if(dist <= err) return true;

        return false;
    }

    private double angleWrap(double radians) {

        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }

        // keep in mind that the result is in radians
        return radians;
    }
}
