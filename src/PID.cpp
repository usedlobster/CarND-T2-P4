#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {

    // save pid constants
    this->Kp = Kp ;
    this->Ki = Ki ;
    this->Kd = Kd ;

    // initialise error totals
    p_error = 0.0 ;
    i_error = 0.0 ;
    d_error = 0.0 ;
}

void PID::UpdateError(double cte) {
    
    d_error  = ( cte - p_error ) ;
    p_error  = cte  ;
    i_error += cte  ;

}

double PID::TotalError() {

    return ( p_error * Kp ) + ( i_error * Ki ) + ( d_error * Kd ) ;
}

// ---------------------------------------
#include <iostream>

PidAutoTuner::PidAutoTuner() {
    // skip first nSkip readings
    nSkip			= 200     ;
    nRun			= 800	 ;

    delta[0] = 1.0		  ;
    delta[1] = 1.0 		  ;
    delta[2] = 1.0 		  ;
    best_error_so_far = -1.0 ;
    twiddle_step  = 1	  ;
    twiddle_param = 0 	  ;

    //
    iter = 0 ;
    error_sum_squared = 0.0 ;




}


bool PidAutoTuner::Update( double e , PID &pid )
{
	iter++ ;
	if ( iter < nSkip )
		return false ;
	else if ( iter == nSkip )
		error_sum_squared = ( e * e ) ;
	else
		error_sum_squared += ( e * e ) ;


	if ( ( best_error_so_far < 0 && error_sum_squared > best_error_so_far  ))
			iter = nSkip + nRun ;
	else if ( iter < nSkip + nRun )
		return false ;	
	
	double p[3] ;
	p[0] = pid.Kp ;
	p[1] = pid.Ki ;
	p[2] = pid.Kd ;
	for(;;)
	{
		if ( twiddle_step == 0 ) {
			p[ twiddle_param ] += delta[ twiddle_param ] ;
			twiddle_step = 1 ;
			break ; 
		} 

		if ( best_error_so_far < 0.0 || error_sum_squared < best_error_so_far ) {
			best_error_so_far = error_sum_squared ;
			delta[ twiddle_param ] *= 1.1 ;
			std::cout << "BEST SO FAR: " << best_error_so_far << ":" << pid.Kp << ":" << pid.Ki << ":" << pid.Kd << std::endl ;
		}
		else 
		{
			if ( twiddle_step == 1 ) 
			{
				p[ twiddle_param ] -= 2.0*delta[ twiddle_param ] ;
				twiddle_step = 2 ;
				break ; // wait for another error estimate
			}
			else		
			{
				p[ twiddle_param ] += delta [ twiddle_param ] ;
				delta[ twiddle_param ] *= 0.9 ;			
			}
		}


		twiddle_param = ( twiddle_param + 1 ) % 3 ;
		twiddle_step = 0 ;

	}
	
	pid.Init( p[0] , p[1] , p[2] ) ;

	iter = 0.0 ;

	return  true ;

}
