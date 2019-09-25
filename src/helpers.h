#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>

// for convenience
using std::string;
using std::vector;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

// get obstacle lane. assuming there are only there lanes with id
// 0,1,2.
int get_obstacle_lane(const float d,
                      const float lane_width){
    int lane = d/lane_width;
    if(lane <= 2){
        return lane;
    }else{
        return -1; // invalid for this project, there are only 3 lanes
    }
}
// check obstacles in car lane
bool check_obstacle_ahead(const int car_lane,
                          const int obstacle_lane,
                          const double car_pos,
                          const double obstacle_pos){
    // if there is any obstcles within pos_buffer ahead, then returns true.
    const double pos_buffer = 20;
    if(car_lane == obstacle_lane){
        if(obstacle_pos > car_pos){
            if(obstacle_pos - car_pos < pos_buffer){
                return true;
            }
        }
    }
    return false;
}

//check obstacle left side
bool check_obstacle_left(const int car_lane,
                         const int obstacle_lane,
                         const double car_pos,
                         const double obstacle_pos){
    // if there is an obstacle which is on left side,
    // less then 20 meters ahead or less then 5 meters behind returns true.
    const double pos_buffer = 20;
    const double pos_buffer_back = 5;
    if(obstacle_lane - car_lane == -1){
        double abs_del = abs(car_pos - obstacle_pos);
        if(abs_del < pos_buffer){
            if(car_pos > obstacle_pos && abs_del > pos_buffer_back){
                return false;
            }
            return true;
        }
    }
    return false;
}

// check obstacle right side
bool check_obstacle_right(const int car_lane,
                          const int obstacle_lane,
                          const double car_pos,
                          const double obstacle_pos){
    // if there is an obstacle which is on right side,
    // less then 20 meters ahead or less then 5 meters behind returns true.
    const double pos_buffer = 20;
    const double pos_buffer_back = 5 ;
    if(obstacle_lane - car_lane == 1){
        double abs_del = abs(car_pos - obstacle_pos);
        if(abs_del < pos_buffer){
            if(car_pos > obstacle_pos && abs_del > pos_buffer_back){
                return false;
            }
            return true;
        }
    }
    return false;
}

// state machine decides which lane to go/follow
// and what speed difference we should apply.
void behavior_state_machine(const bool obs_ahead,
                            const bool obs_left,
                            const bool obs_right,
                            //const double ref_vel,
                            int& lane,
                            double& del_vel){
    // acceleration which controls change in velocity.
    // maximum accelertaion can be 0.224. 0.75 factor is applied here.
    double del_a = 0.224*0.75;
    if(obs_ahead == true){
        if(lane != 0 && obs_left == false){
            lane-=1; // lane change left
        }else if(lane != 2 && obs_right == false){
            lane+=1; // lane change right
        }else{
            // lane change is not possible.
            // so do lane follow and slow down.
            del_vel-= del_a;
        }
    }else{
        // this means there is no obstacle ahead. So no need to change the lane.
        // and we can increase the velocity.
        del_vel+= del_a;
    }
}

#endif  // HELPERS_H
