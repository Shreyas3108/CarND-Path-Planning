# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Reflection

The car drives well beyond 10 miles. The path planning algorithm is coded on the main.cpp inside the src folder. The code consists of prediction of waypoints that is how car has to switch the lane when a car is ahead while checking the others lane. 
We can see the code here , 

### 1. Prediction of other cars 

In this part we check whether the other car is in our lane and if in case it is in our lane then we try to switch to other lane , We also check if car is in the other lane or not. We also want to set our speed in case other cars are present in the other lanes and we cannot make the switch. Hence in easier words , We are looking out for the two conditions and code our vehicle to act accodingly , 

1. Check if the car is present in the same lane and if so try to make switch. 
2. If the car is present in the other lane too then don't switch. In case the speed of the car ahead is slower than ours , We need to slow down. 
```cpp 
			//initializing no car is ahead , Here We assume that no car is ahead and the variables value then change depending upon the scenario 
			// this is very important as the car characterisitic would depend upon this. 
            bool car_ahead = false;
            bool car_left = false;
            bool car_right = false; 
			
			//checking for cars in same lane 
			for(int i = 0 ; i < sensor_fusion.size() ; i++)
			{
				float d_coord = sensor_fusion[i][6] ; 
				int car_init_lane = -1 ;  // Initially we assume that car's aren't present 
				
				//defining the lanes through size , as mentioned on the walkthrough each lane has 4 meter length. 
				// after which , we assume where the cars are. 
				if(d_coord > 0 && d_coord < 4)
				{
					car_init_lane = 0 ; 
				}
				else if(d_coord > 4 && d_coord <8) 
				{
					car_init_lane = 1 ; 
				}
				else if(d_coord > 8 && d_coord < 12) 
				{
					car_init_lane = 2 ; 
				}
				else  
				{
					continue ; 
				} 
				
				// Checking the speed of the other car in order for us to slow down/fasten up. 
				double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
				double check_car_s = sensor_fusion[i][5];
                double check_speed = sqrt(pow(vx, 2) + pow(vy, 2));
				check_car_s += ((double)prev_size*0.02*check_speed); // How far is the other car ? 
				
				// defining the car behaviour 
				//if(car_init_lane = lane) //check car in the same lane
				//{
				//	if(check_car_s > car_s && check_car_s - car_s < 30)
				//	{
				//		car_ahead = true ; 
				//	}
				//}
				//else if(car_init_lane = lane - 1) // is it on the left ? 
				//{
				//	if(car_s - 30 < check_car_s && car_s + 30 > check_car_s)
				//	{
				//		car_left = true; 
				//	}
				//}
				//else if(car_init_lane = lane+1) // how about right ? 
				//{
				//	if(car_s - 30 < check_car_s && car_s + 30 > check_car_s)
				//	{
				//		car_right = true;
				//	}
				//} 
				
				// Defining the car behaviour and setting the boolean true for cases when the car is ahead and the gap is less than 30 meters  
				if (car_init_lane == lane) {
					// Another car is ahead
					car_ahead |= (check_car_s > car_s) && ((check_car_s - car_s) < 30);
				} else if (car_init_lane - lane == 1) {
					// Another car is to the right
					car_right |= ((car_s - 30) < check_car_s) && ((car_s + 30) > check_car_s);
				} else if (lane - car_init_lane == 1) {
					// Another car is to the left
					car_left |= ((car_s - 30) < check_car_s) && ((car_s + 30) > check_car_s);
				}
			}
         	if(car_ahead) //If car is ahead then , We must chane , This scope helps us do so. We also check if the lane is available or not 
			{
				if ( !car_left && lane > 0 ) 
				{
                lane--; // Move to the left lane is it's empty  
				} 
			  else if ( !car_right && lane != 2 )
				{
				lane++; // Move to the right lane if it is empty. 
				} 
			  else 
			  {
                s_diff -= max_acc; //deaccelerate so that there is no collision 
              }
            } 
			else 
			{

              if ( lane != 1 ) 
			  { 
                if ( ( lane == 0 && !car_right ) || ( lane == 2 && !car_left ) )
				{
                  lane = 1; // Back to center.
                }
              }
              if ( ref_vel < max_vel ) 
			  {
                s_diff += max_acc; // accelerate if we are far away 
              }
			}
``` 

### 2. Make a trajectory for the car to move. 

We initialize the data from the sensor and create a trajectory which the car would follow , This trajectory would change in case of car which goes without saying. 
This part depends exactly on the previous calculations , Which helps the car create paths.
We always won't have the data and in our case if the car hasn't moved for more than 60 meters the system uses the car's current position instead of the previous waypoints. The Frenet helper function crates three points at an interval of 30 meters in front of the car
We use splines to generate the trajectory, a shift and rotate transform is applied.

We generate 50 waypoints , Since we don't want more waypoints which would be a problem in case of sudden change and neither do we need a smaller waypoint which would hamper the trajectory. Hence the car generates 50 waypoints ahead which helps in the movement of the car in forward direction. 

Here's the code 

```cpp 
        	vector<double> x_vals;
          	vector<double> y_vals;
			double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw); 
			//Checking for previous data points 
			if ( prev_size < 2 ) {
                double prev_car_x = car_x - cos(car_yaw);
                double prev_car_y = car_y - sin(car_yaw);

                x_vals.push_back(prev_car_x);
                x_vals.push_back(car_x);

                y_vals.push_back(prev_car_y);
                y_vals.push_back(car_y);

            } else // use the last two data points. 
			{
				
                ref_x = previous_path_x[prev_size - 1];
                ref_y = previous_path_y[prev_size - 1];

                double ref_x_prev = previous_path_x[prev_size - 2];
                double ref_y_prev = previous_path_y[prev_size - 2];

                ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

                x_vals.push_back(ref_x_prev);
                x_vals.push_back(ref_x);
                y_vals.push_back(ref_y_prev);
                y_vals.push_back(ref_y);
            }
          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

			vector<double> wp0_nxt = getXY(car_s + 30, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
			vector<double> wp1_nxt = getXY(car_s + 60, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
			vector<double> wp2_nxt = getXY(car_s + 90, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);


            x_vals.push_back(wp0_nxt[0]);
            x_vals.push_back(wp1_nxt[0]);
            x_vals.push_back(wp2_nxt[0]);
			
            y_vals.push_back(wp0_nxt[1]);
            y_vals.push_back(wp1_nxt[1]);
            y_vals.push_back(wp2_nxt[1]);
			// Coordinating the car coordinates
            for ( int i = 0; i < x_vals.size(); i++ )
				{
				double shift_x = x_vals[i] - ref_x;
				double shift_y = y_vals[i] - ref_y;
				x_vals[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
				y_vals[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
            }
            // Create the spline.
            tk::spline s;
            s.set_points(x_vals , y_vals);
			 // Output path from previous iterations.
          	vector<double> next_x_vals;
          	vector<double> next_y_vals;
            for ( int i = 0; i < prev_size; i++ ) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }
            // distance y position
            double target_x = 30.0;
            double target_y = s(target_x);
            double target_dist = sqrt(target_x*target_x + target_y*target_y);
            double x_add_on = 0;
            for( int i = 1; i < 50 - prev_size; i++ ) { // 50 way points - Size from prev iteration 
              ref_vel += s_diff;
              if ( ref_vel > max_vel ) {
                ref_vel = max_vel;
              } else if ( ref_vel < max_acc ) {
                ref_vel = max_acc;
              }
              double N = target_dist/(0.02*ref_vel/2.24);
              double x_point = x_add_on + target_x/N;
              double y_point = s(x_point);
              x_add_on = x_point;
              double x_ref = x_point;
              double y_ref = y_point;
              x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
              y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);
              x_point += ref_x;
              y_point += ref_y;
              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }
```  

## Final Screenshots 

Here's how the lane change works in the simulator , 

![Car Changing lane](https://raw.githubusercontent.com/Shreyas3108/CarND-Path-Planning/master/Screenshot%20(1302).png) 

The car as seen above is changing the lane. 

Here's the milestone accomplishment of more than 4.32 miles as required in the rubrics. 

![Milestone!](https://raw.githubusercontent.com/Shreyas3108/CarND-Path-Planning/master/Screenshot%20(1304).png)


## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

