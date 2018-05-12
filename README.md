# Introductions
This is the last project of self driving car nano degree, it used model predictive control methology to deal with actuation command latency convert the following trajectory as an optimization problem.


# Describe MPC Model
MPC uses an optimizer to find the control inputs and minimize  the cost function. First we pass the current state to the model prredictive controller,  and then the optimization solver will be called. The solver uses the initial state, the model constraints and cost function to return a vector of control inputs that could minimize the cost function. After apply the first control input, it will repeat the loop again.

Update equation is as follow:

x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
v_[t+1] = v[t] + a[t] * dt
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt


# Timestep Length and Elapsed Duration (N & dt)
To get the duration of future perdiction, we beed N and dt. N is the number of time stamps and dt is the time elapse between actuations.

Because the latency is 0.1 second, so I set the dt as 0.1 as well. Initially I set N with 25 and dt as 0.1,  but the car is nor stable so that i lower the N to 12 and dt as 0.1, then it works fine at a lower reference speed at 35.



# Polynomial Fitting and MPC Preprocessing
First the waypoints need to be tranformed from global point to car coordinate, this can be achived by:

#transform to map x coordinate
x_map= x_part + (np.cos(theta) * x_obs) - (np.sin(theta) * y_obs)

#transform to map y coordinate
y_map= y_part + (np.sin(theta) * x_obs) + (np.cos(theta) * y_obs)


We use `polyfit` to fit a third order polynomial to the (x, y) coordinates.



# Model Predictive Control with Latency
In the real car, the actuation command won't execute immediatelly and there is usually a small delay. To deal with the latency issue, one way is to simulate the vehicle model starting from the current state for the duration of latency. So in every loop I use the simulated result state as the new initial state for MPC model.



# Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

# Result Video
Result video can be seen from here:
[link to p5.mov](./p5.mov)
