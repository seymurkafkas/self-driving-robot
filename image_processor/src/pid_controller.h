#include <iostream>

class PID_Controller
{
    double Kp, Kd, Ki;                   //proportional ,derivative and integral constants
    double maxIntegrator, minIntegrator; // upper and lower bounds
    double previousError, integrator;
    double maxAngularVelocity, minAngularVelocity;

public:
    PID_Controller()
    {
        Kp = 0.012;
        Kd = 0.002;
        Ki = 0.001;
        maxIntegrator = 200;
        minIntegrator = -200;
        previousError = 0.0;
        integrator = 0.0;

        maxAngularVelocity = 66;
        minAngularVelocity = -66;
    }
    ~PID_Controller()
    {
    }

    double getPIDOutput(double newError)
    {
        double proportionalComponent = Kp * newError;
        double derivativeComponent = Kd * (newError - previousError);
        previousError = newError;
        integrator += newError;
        if (integrator > maxIntegrator)
        {
            integrator = maxIntegrator;
        }
        else if (integrator < minIntegrator)
        {
            integrator = minIntegrator;
        }
        double integralComponent = Ki * integrator;

        double totalOutput = proportionalComponent + integralComponent + derivativeComponent;

        //std::cout<<totalOutput<<std::endl;
        return std::min(maxAngularVelocity, std::max(minAngularVelocity, totalOutput));
    }
};
