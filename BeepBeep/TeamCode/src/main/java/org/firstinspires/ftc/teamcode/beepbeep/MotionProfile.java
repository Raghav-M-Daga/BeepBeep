package org.firstinspires.ftc.teamcode.beepbeep;

public class MotionProfile {
    private double maxAcceleration;
    private double maxVelocity;
    private double accelerationDt;
    private double accelerationDistance;
    private double deaccelerationDt;
    private double cruiseDt;

    public MotionProfile(double maxAcceleration, double maxVelocity, double distance) {
        this.maxAcceleration = maxAcceleration;
        this.maxVelocity = maxVelocity;

        // Calculate the time it takes to accelerate to max velocity
        this.accelerationDt = maxVelocity / maxAcceleration;

        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
        double halfwayDistance = distance / 2;
        this.accelerationDistance = 0.5 * maxAcceleration * Math.pow(accelerationDt, 2);

        if (this.accelerationDistance > halfwayDistance) {
            this.accelerationDt = Math.sqrt(halfwayDistance / (0.5 * maxAcceleration));
        }

        this.accelerationDistance = 0.5 * maxAcceleration * Math.pow(accelerationDt, 2);

        // Recalculate max velocity based on the time we have to accelerate and decelerate
        this.maxVelocity = maxAcceleration * accelerationDt;

        // We decelerate at the same rate as we accelerate
        this.deaccelerationDt = accelerationDt;

        // Calculate the time that we're at max velocity
        double cruiseDistance = distance - 2 * accelerationDistance;
        this.cruiseDt = cruiseDistance / maxVelocity;
    }

    public double getPosition(double elapsedTime) {
        // Check if we're still in the motion profile
        double entireDt = accelerationDt + cruiseDt + deaccelerationDt;
        if (elapsedTime > entireDt) {
            return getTotalDistance();
        }

        // If we're accelerating
        if (elapsedTime < accelerationDt) {
            // Use the kinematic equation for acceleration
            return 0.5 * maxAcceleration * Math.pow(elapsedTime, 2);
        }

        // If we're cruising
        else if (elapsedTime < accelerationDt + cruiseDt) {
            double cruiseCurrentDt = elapsedTime - accelerationDt;

            // Use the kinematic equation for constant velocity
            return accelerationDistance + maxVelocity * cruiseCurrentDt;
        }

        // If we're decelerating
        else {
            double deaccelerationTime = elapsedTime - (accelerationDt + cruiseDt);

            // Use the kinematic equations to calculate the instantaneous desired position
            return accelerationDistance + (maxVelocity * cruiseDt) + (maxVelocity * deaccelerationTime) - 0.5 * maxAcceleration * Math.pow(deaccelerationTime, 2);
        }
    }

    public double getVelocity(double elapsedTime) {
        // Check if we're still in the motion profile
        double entireDt = accelerationDt + cruiseDt + deaccelerationDt;
        if (elapsedTime > entireDt) {
            return 0.0; // We have completed the motion profile, so velocity is 0.
        }

        // If we're accelerating
        if (elapsedTime < accelerationDt) {
            // Use the kinematic equation for acceleration to calculate velocity
            return maxAcceleration * elapsedTime;
        }

        // If we're cruising
        else if (elapsedTime < accelerationDt + cruiseDt) {
            return maxVelocity;
        }

        // If we're decelerating
        else {
            double deaccelerationTime = elapsedTime - (accelerationDt + cruiseDt);

            // Use the kinematic equations to calculate the instantaneous desired velocity
            return maxVelocity - maxAcceleration * deaccelerationTime;
        }
    }

    public double getAcceleration(double elapsedTime) {
        // Check if we're still in the motion profile
        double entireDt = accelerationDt + cruiseDt + deaccelerationDt;
        if (elapsedTime > entireDt) {
            return 0.0; // We have completed the motion profile, so acceleration is 0.
        }

        // If we're accelerating
        if (elapsedTime < accelerationDt) {
            return maxAcceleration;
        }

        // If we're cruising
        else if (elapsedTime < accelerationDt + cruiseDt) {
            return 0.0; // No acceleration during the cruising phase.
        }

        // If we're decelerating
        else {
            return -maxAcceleration;
        }
    }

    public double getTotalTime() {return accelerationDt + cruiseDt + deaccelerationDt;}

    public double getTotalDistance() {
        return 2 * accelerationDistance + maxVelocity * cruiseDt;
    }

    public boolean isFinished(double elapsedTime) {
        // Check if we're still in the motion profile
        double entireDt = accelerationDt + cruiseDt + deaccelerationDt;
        if (elapsedTime > entireDt) {
            return true; // We have completed the motion profile, so velocity is 0.
        }

        return false;
    }
}
