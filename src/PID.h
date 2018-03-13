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

    /*
    * Coefficients
    */
    double Kp;
    double Ki;
    double Kd;

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
};


class PidAutoTuner {
  private:
    int iter  ;
    int nSkip ;
    int nRun  ;
    int twiddle_step          ;
    int twiddle_param         ;
    double error_sum_squared  ;
    double err                ;
    double best_error_so_far  ;
    double delta[3]           ;
    double delta_sum          ;
  private:
    void StartTune() ;
  public:
    PidAutoTuner() ;
    bool Update( double e , PID &pid ) ;
} ;





#endif /* PID_H */
