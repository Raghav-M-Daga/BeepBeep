package org.firstinspires.ftc.teamcode.beepbeep;

import static org.firstinspires.ftc.teamcode.beepbeep.BeepDriveConstants.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.beepbeeplib.util.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pointfollow.BezierCurve;

import java.util.function.Consumer;

public class Trajectory {
    SampleMecanumDrive drive;
    String trajType = "";
    PIDController pid_x = new PIDController(Kp_x, Ki_x, Kd_x);
    PIDController pid_y = new PIDController(Kp_y, Ki_y, Kd_y);
    PIDController pid_heading = new PIDController(Kp_heading, Ki_heading, Kd_heading);
    
    BezierCurveCalc bezierCalc = new BezierCurveCalc();
    BezierCurve curve_x, curve_y;
    private double curveLength, curve_heading;

    MotionProfile motionProfile;
    Consumer<Double> method;

    double control_signal_x, control_signal_y, control_signal_heading = 0;
    double instantTargetPosition, u = 0;
    double instantTargetVelocity, duds, dxdu, dydu, x_vel, y_vel = 0;
    double instantTargetAcceleration, du2ds, dx2du, dy2du, x_accel, y_accel = 0;
    double totalY, totalX = 0;

    public Trajectory(SampleMecanumDrive drive, BezierCurve x, BezierCurve y, double heading) {
        this.drive = drive;
        this.trajType = "bezier";
        this.curveLength = bezierCalc.bezier_length(x, y);
        this.curve_x = x;
        this.curve_y = y;
        this.curve_heading = heading;
        this.motionProfile = new MotionProfile(maxAccel, maxVel, this.curveLength);
        this.method = (time) -> { followBezier(time); };
    }

    public void follow(Double time) {
        this.method.accept(time);
    }


    public void followBezier(double time) {
        Pose2d poseEstimate = this.drive.getPoseEstimate();
        ElapsedTime timer = new ElapsedTime();

        // Position
        instantTargetPosition = this.motionProfile.getPosition(timer.time());
        u = bezierCalc.bezier_param_of_disp(instantTargetPosition, bezierCalc.get_sums(), bezierCalc.get_upsilon());
        control_signal_x = pid_x.calculate(this.curve_x.bezier_get(u), poseEstimate.getX());
        control_signal_y = pid_y.calculate(this.curve_y.bezier_get(u), poseEstimate.getY());
        control_signal_heading = pid_heading.calculate(angleWrap(Math.toRadians(this.curve_heading)), angleWrap(poseEstimate.getHeading()));

        // Velocity
        // s′(t)u′(s(t))x′(u(s(t)))
        instantTargetVelocity = this.motionProfile.getVelocity(timer.time());
        duds = bezierCalc.bezier_param_of_disp_deriv(this.curve_x, this.curve_y, u);
        dxdu = this.curve_x.bezier_deriv(u);
        dydu = this.curve_y.bezier_deriv(u);
        x_vel = dxdu * duds * instantTargetVelocity;
        y_vel = dydu * duds * instantTargetVelocity;

        // Acceleration
        // (s'(t))^2 * (u'(s(t)))^2 * x''(u(s(t))) + ((s'(t))^2 * u''(s(t)) + s''(t) * u'(s(t))) * x'(u(s(t)))
        instantTargetAcceleration = this.motionProfile.getAcceleration(timer.time());
        du2ds = bezierCalc.bezier_param_of_disp_deriv2(this.curve_x, this.curve_y, u);
        dx2du = this.curve_x.bezier_deriv2(u);
        dy2du = this.curve_y.bezier_deriv2(u);
        x_accel = Math.pow(instantTargetVelocity, 2) * Math.pow(duds, 2) * dx2du + Math.pow(instantTargetVelocity, 2) * du2ds + instantTargetAcceleration * duds * dxdu;
        y_accel = Math.pow(instantTargetVelocity, 2) * Math.pow(duds, 2) * dy2du + Math.pow(instantTargetVelocity, 2) * du2ds + instantTargetAcceleration * duds * dydu;

        // Combined
        totalX = control_signal_x + x_vel * kV + x_accel * kS;
        totalY = control_signal_y + y_vel * kV + y_accel * kS;

        Vector2d input = new Vector2d(
                totalX,
                totalY
        ).rotated(-poseEstimate.getHeading());

        this.drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        control_signal_heading
                )
        );

        this.drive.update();
    }

    private void followLinear(double time) {

    }

    public double angleWrap(double radians) {

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
