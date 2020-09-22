#include "multi_controller.h"
//#include "fl/Headers.h"

using namespace std;

#define PI 3.14159265

controller::controller()
{
	this->V_max = 0;
	this->W_max = 0;
	this->safety_R = 0.5;
	
	this->V_input = 0;
	this->omega_input = 0;
}

controller::~controller() {}

geometry_msgs::Twist controller::Simple_P_controller(double Re, double Te)
{
	cout << "Here is a P controller" << endl;
	double Kp_V = 1;
	this->V_input = Re*Kp_V;  // use R error to generate the command signal
	
	double Kp_Omega = 1;
	this->omega_input = Te*Kp_Omega;   // use theta error to generate the command signal
	
	controller::limitation();

	return controller::command_publish();
}


geometry_msgs::Twist controller::Simple_PD_controller(double Re, double Te, double Re_pre, double Te_pre)
{
	cout << "Here is a PD controller" << endl;
	double Kp_V = 1;
	double Kd_V = 10;
	this->V_input = Re*Kp_V + (Re-Re_pre)*Kd_V;  // use R error to generate the command signal
	
	double Kp_Omega = 1;
	double Kd_Omega = 10;
	this->omega_input = Te*Kp_Omega + (Te-Te_pre)*Kd_Omega;    // use theta error to generate the command signal
	
	controller::limitation();
	
	return controller::command_publish();
}


geometry_msgs::Twist controller::Improved_P_controller(double Re, double Te, double people_relative_linear_velocity, double robot_linear_velocity)
{
	cout << "Here is a improved P controller" << endl; 
	double Kp_V = 1;
	this->V_input = people_relative_linear_velocity + Re*Kp_V + robot_linear_velocity*0.6;  // Max speed saturation      
	
	double Kp_Omega = 1;
	this->omega_input = Te*Kp_Omega;
	
	controller::limitation();
	
	return controller::command_publish();
}

geometry_msgs::Twist controller::Nonlinear_controller_Feedback_linearization(double people_x, double people_y, double people_relative_linear_velocity, double people_relative_shift_velocity, double robot_linear_velocity)
{
	cout << "Here is a feedback linearization controller" << endl;
	double k1 = 1;
	double k2 = 1;
	this->V_input = (people_relative_linear_velocity+robot_linear_velocity*0.6) + k1*(people_y-safety_R) + (-people_x)*(k2*(-people_x)-people_relative_shift_velocity)/people_y;
	this->omega_input = (k2*(-people_x)-people_relative_shift_velocity)/people_y;
	
	controller::limitation();
	
	return controller::command_publish();
}

geometry_msgs::Twist controller::Sigmoid_function_controller(double Re, double Te)
{
	cout << "Here is a Sigmoid controller" << endl; 
	double a_linear = 3;
	this->V_input = this->V_max*(2./(1+exp(-a_linear*Re))-1);
	double a_angular = 5;
	this->omega_input = this->W_max*(2./(1+exp(-a_angular*Te))-1);
	
	controller::limitation();
	
	return controller::command_publish();
}


geometry_msgs::Twist controller::Improved_double_Sigmoid_controller(double Re, double Te)
{
	cout << "Here is a improved Sigmoid controller" << endl; 
	double a_linear = 7;
	if (Re>0)
	{
		this->V_input = this->V_max*(1./(1+exp(-a_linear*(Re-0.3))));
		this->V_input = this->V_input - this->V_max*(1./(1+exp(-a_linear*(0-0.3))));
	}
	else
		this->V_input = 0;

	double a_angular = 6;
	if (Te<0)
	{
		this->omega_input = this->W_max*(1./(1+exp(-a_angular*(Te+35*PI/180)))-1);
		this->omega_input = this->omega_input-this->W_max*(1./(1+exp(-a_angular*(0+35*PI/180)))-1);
	}
	else  // Te >=0
	{
		this->omega_input = this->W_max*(1./(1+exp(-a_angular*(Te-35*PI/180))));
		this->omega_input = this->omega_input-this->W_max*(1./(1+exp(-a_angular*(0-35*PI/180))));
	}
	
	controller::limitation();
	
	return controller::command_publish();
}


/*geometry_msgs::Twist controller::Fuzzy_controller(double Re, double Te)
{
	fl::Engine* engine1 = fl::FisImporter().fromFile("/home/kevinlee/catkin_ws/src/path_planning/src/fuzzy/AGV1.fis");
	fl::Engine* engine2 = fl::FisImporter().fromFile("/home/kevinlee/catkin_ws/src/path_planning/src/fuzzy/AGV2.fis");
	cout << "Here is a Fuzzy controller" << endl; 

	if (Re > 0)
	{
		engine1->setInputValue("R_err",Re);
		engine1->setInputValue("theta_err",fabs(Te));
		engine1->process();
		this->V_input = engine1->getOutputValue("Vc");
	}
	else
		this->V_input = 0;

	engine2->setInputValue("theta_err",Te);
	engine2->process();
	this->omega_input = engine2->getOutputValue("Wc");
	
	return controller::command_publish();
}*/

geometry_msgs::Twist controller::MPC_controller(double Re, double Te)
{
	cout << "Here is a MPC controller" << endl; 
	double Ts = 0.5;  // sampling time (s)  Ts=0.2
	vector<double> Vc = {};
	for (float i = 0; i < 1; i+=0.01) { Vc.push_back(i); };    // 0.05
	vector<double> Wc = {};
	for (int i = -50; i<=50; i+=2)	{ Wc.push_back(i*PI/180); }     // 10
	vector<double> Target_cost = {};
	
	for (auto& iter1 : Vc)
	{
		for (auto& iter2 : Wc)
		{
			double R_predict = sqrt(pow(Re,2) + pow((iter1)*Ts,2) - 2*Re*(iter1)*Ts*cos(Te - (iter2)*Ts));
			double R_delta = fabs(R_predict);
			double W_delta = fabs(Te - (iter2)*Ts);
			//cout << "(R_delta,W_delta) = (" << R_delta << "," <<  W_delta << ")" << endl;
			Target_cost.push_back(1*R_delta + 1*W_delta);
		}
	}
	
	auto minElementIndex = std::min_element(Target_cost.begin(),Target_cost.end()) - Target_cost.begin();
	double minElement = *std::min_element(Target_cost.begin(), Target_cost.end());
	cout << "Min = " << minElement << ", Index = " << minElementIndex << endl;
	int min_row = minElementIndex/Wc.size();
	int min_col = minElementIndex%Wc.size();
	this->V_input = Vc[min_row];
	this->omega_input = Wc[min_col];
		
	return controller::command_publish();
}

void controller::limitation()
{
	if (this->V_input > this->V_max)
		this->V_input = this->V_max;
	if (this->V_input < 0)  // prevent go backward
		this->V_input = 0;

	if (fabs(this->omega_input) > this->W_max)  // Max angular saturation
		this->omega_input = (this->omega_input>0) ? this->W_max : -this->W_max;
	
	return;
}

geometry_msgs::Twist controller::command_publish()
{
	geometry_msgs::Twist velocity_input;  // capsualize the system input 
	velocity_input.linear.x = this->V_input;
	velocity_input.angular.z = this->omega_input;
	return velocity_input;
}

