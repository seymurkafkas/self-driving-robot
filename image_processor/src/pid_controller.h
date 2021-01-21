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
        Kp = 0.01;
        Kd = 0.0004;
        Ki = 0.0001;
        maxIntegrator = 200;
        minIntegrator = -200;
        previousError = 0.0;
        integrator = 0.0;

        maxAngularVelocity = 3;
        minAngularVelocity = -3;
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

        //   std::cout << "Proportional comp: " << proportionalComponent << std::endl;
        //   std::cout << "Integral comp: " << integralComponent << std::endl;
        //  std::cout << "Derivative comp: " << derivativeComponent << std::endl;

        double totalOutput = proportionalComponent + integralComponent + derivativeComponent;

        if (totalOutput > 1 || totalOutput < -1)
        {
            std::cout << "--------------------" << std::endl;

            if (fabs(integralComponent) == std::max(fabs(integralComponent), std::max(fabs(derivativeComponent), fabs(proportionalComponent))))
            {

                std::cout << "Integral" << std::endl;
            }
            else if (fabs(proportionalComponent) == std::max(fabs(proportionalComponent), fabs(derivativeComponent)))
            {

                std::cout << "Proportional" << std::endl;
            }
            else
            {
                std::cout << "Derivative: " << derivativeComponent << std::endl;
            }
        }

        //std::cout<<totalOutput<<std::endl;
        return std::min(maxAngularVelocity, std::max(minAngularVelocity, totalOutput));
    }
};
