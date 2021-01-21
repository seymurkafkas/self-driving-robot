
class PID_Controller
{
	double Kp, Kd, Ki;					 //proportional ,derivative and integral constants
	double maxIntegrator, minIntegrator; // upper and lower bounds
	double previousError, integrator;

public:
	PID_Controller()
	{
		Kp = 0.009;
		Kd = 0.01;
		Ki = 0;
		maxIntegrator = 25;
		minIntegrator = -25;
		previousError = 0.0;
		integrator = 0.0;
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
		return proportionalComponent + integralComponent + derivativeComponent;
	}
};
