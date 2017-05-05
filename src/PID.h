#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  double err;
  double best_err;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;
  double dpp;
  double dpi;
  double dpd;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
  /*
  * Calculate the twddle.
  */
  void Twiddle();
  int nb_frames;
private:

  bool go_down;

  int pid_index;
  int min_frames;
};

#endif /* PID_H */
