#include <iostream>

class PIDController
{
    double Kp, Kd, Ki;                   //proportional ,derivative and integral constants
    double maxIntegrator, minIntegrator; // upper and lower bounds
    double previousError, integrator;
    double maxAngularVelocity, minAngularVelocity;

public:
    PIDController()
    {
        Kp = 0.009;
        Kd = 0.0015;
        Ki = 0.0005;
        maxIntegrator = 200;
        minIntegrator = -200;
        previousError = 0.0;
        integrator = 0.0;

        maxAngularVelocity = 2;
        minAngularVelocity = -2;
    }
    ~PIDController()
    {
    }

    double getPIDOutput(double newError)
    {
        double proportionalComponent = Kp * newError;
        double derivativeComponent = Kd * (newError - previousError);
        previousError = newError;
        integrator += newError;
        if (integrator > maxIntegrator)
            integrator = maxIntegrator;
        else if (integrator < minIntegrator)
            integrator = minIntegrator;
        double integralComponent = Ki * integrator;
        double totalOutput = proportionalComponent + integralComponent + derivativeComponent;
        return std::min(maxAngularVelocity, std::max(minAngularVelocity, totalOutput));
    }
};
