#include "PID.h"
#include <limits>
#include <math.h>
#include <iostream>
using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
	p_error = 0;
	i_error = 0;
	d_error = 0;
	dpp = 0.1;
	dpi = 0.01;
	dpd = 1;
	nb_frames = 0;
	min_frames = 50;
	err = 0;
	best_err = numeric_limits<float>::max();
	pid_index = 0;
	go_down = false;
}

void PID::UpdateError(double cte) {
	d_error = cte - p_error;
	p_error = cte;
	i_error += cte;
//	if (nb_frames >= 2*min_frames){
//		Twiddle();
//		d_error = 0;
//		p_error = 0;
//		i_error = 0;
//		nb_frames -= 2*min_frames;
//	}
//	if (nb_frames >= min_frames){
//		err += pow(cte,2);
//	}
//	nb_frames += 1;
}

double PID::TotalError() {
	return - Kp * p_error - Ki * i_error - Kd * d_error;
}

void PID::Twiddle() {
	double *p;
	double *dp;
	err /= min_frames;
	if (pid_index == 0){
		p = &Kp;
		dp = &dpp;
	}
	else if (pid_index == 1){
		p = &Ki;
		dp = &dpi;
	}
	else {
		p = &Kd;
		dp = &dpd;
	}
	if (err < best_err){
		best_err = err;
		*dp *= 1.1;
		pid_index = (pid_index + 1) % 3;
		go_down = false;
	}
	else{
		if (go_down){
			*p += *dp;
			*dp *= 0.9;
			pid_index = (pid_index + 1) % 3;
			go_down = false;
		}
		else{
			go_down = true;
		}
	}
	if (go_down){
		*p -= 2**dp;
	}
	else{
		*p += *dp;
	}
	err = 0;
}

