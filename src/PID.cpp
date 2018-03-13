#include "PID.h"
// include for debugging.
#include <iostream>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {

    // save given pid constants
    this->Kp = Kp ;
    this->Ki = Ki ;
    this->Kd = Kd ;

    // initialise error totals to zero
    p_error = 0.0 ;
    i_error = 0.0 ;
    d_error = 0.0 ;
}

void PID::UpdateError(double cte) {
    // d_error is current - previous
    d_error  = ( cte - p_error ) ;
    // just current error
    p_error  = cte  ;
    // sum of errors
    i_error += cte  ;

}

// return total error multipled by respective gains.
double PID::TotalError() {

    return ( p_error * Kp ) + ( i_error * Ki ) + ( d_error * Kd ) ;
}


PidAutoTuner::PidAutoTuner() {
    // skip first nSkip readings
    nSkip			= 200    ;
    nRun			= 1800	 ;

    delta[0] = 1.0		  ;
    delta[1] = 1.0		  ;
    delta[2] = 1.0		  ;
    best_error_so_far = 1e100 ;
    twiddle_step  = 0	  ;
    twiddle_param = 0 	  ;

    //
    iter = 0 ;
    error_sum_squared = 0.0 ;

}


bool PidAutoTuner::Update( double e, PID &pid ) {

    iter++ ;
    // skip  the first nSkip samples
    if ( iter < nSkip )
        return false ;


    if ( iter == nSkip )
        error_sum_squared = e ; // first sample
    else if ( ( e < -5.0 ) && ( e > 5.0 ))
        error_sum_squared = best_error_so_far + 1.0 ; // force opt out early if e is between these
    else
        error_sum_squared += e ; //   sum error so far  ( we assume only passed e > 0 )


    // have we twiddled enougth
    if ( ( delta[0] + delta[1] + delta[2] ) < 1e-3 ) {
        iter = 0 ;
        std::cout << "-DONE-" << std::endl ;
        return  false ;
    }

    if ( error_sum_squared > best_error_so_far  )
        iter = nSkip + nRun ; // skip summing error and go to next twiddle step.
    else if ( iter < nSkip + nRun )
        return false ;	// keep collection samples


    double p[3] ;

    p[0] = pid.Kp ;
    p[1] = pid.Ki ;
    p[2] = pid.Kd ;

    for(;;) {
        if ( twiddle_step == 0 ) {
            p[ twiddle_param ] += delta[ twiddle_param ] ;
            twiddle_step = 1 ;
            break ;
        }

        if ( error_sum_squared < best_error_so_far ) {
            best_error_so_far = error_sum_squared ;
            delta[ twiddle_param ] *= 1.1 ;
            std::cout << "BEST SO FAR: " << best_error_so_far << ":" << pid.Kp << ":" << pid.Ki << ":" << pid.Kd << std::endl ;
        } else {
            if ( twiddle_step == 1 ) {
                p[ twiddle_param ] -= 2.0*delta[ twiddle_param ] ;
                twiddle_step = 2 ;
                break ; // wait for another error estimate
            } else {
                p[ twiddle_param ] += delta [ twiddle_param ] ;
                delta[ twiddle_param ] *= 0.9 ;
            }
        }


        twiddle_param = ( twiddle_param + 1 ) % 3 ;
        twiddle_step = 0 ;

    }

    // change pid gains
    pid.Init( p[0], p[1], p[2] ) ;
    // reset step count
    iter = 0.0 ;
    //
    return  true ;

}
