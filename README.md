# CarND-Controls-PI
Self-Driving Car Engineer Nanodegree Program

## Reflection

#### *Describe the effect each of the P, I, D components had in your implementation.*
 
We have implemented a classic Proportional-Integral-Derivative or PID controller in C++ code. The PID controller is constantly been given the cross track error ( cte ) from the simulator.The PID controller  calculates P , I and D responses based on the current error , prev


* The P-Term - is the "Proportional" term, and as its name implies causes the car to steer in proportion to the current error. If we set the Kp gain too high the car will overshoot the required trajectory. If too low then the car does not respond to fast changing cross track errors , such as when going round a bend. The chart below shows the cross track error for the first 250 steps, with a Kp of 0.2 and 0.1 .respectively showing that the higher Kp has a higher frequency and amplitude as expected.

![p-plot](images/plot-p.png)


* The I-Term - is the "Integral" term.  The I-Term takes into account the sum of all previous errors. So if the car is driving to say the left of the correct path - then over time the I error sum will accumulate to such a point that the steering is forced to the right. 

![i-plot](images/plot-i.png)
* The D-Term

![f-plot](images/plot-d.png)

=
