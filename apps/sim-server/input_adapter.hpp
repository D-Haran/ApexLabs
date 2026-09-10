#pragma once
#include <algorithm>
#include <cmath>
// Device adapter, deliberately outside the vehicle model. No yaw/slip feedback.
struct KeyboardParameters {
 double rise=1.5, release=2.5, throttle_rise=1.5, throttle_release=3., brake_rise=3., brake_release=5.;
 double low_speed_angle=.65, high_speed_acceleration=5.;
};
struct KeyboardInput {
 double steer=0,throttle=0,brake=0;
 static double approach(double old,double target,double rate,double dt){return old+std::clamp(target-old,-rate*dt,rate*dt);}
 static double angle_limit(double speed,double wheelbase,const KeyboardParameters& p){
  return std::min(p.low_speed_angle,std::atan(p.high_speed_acceleration*wheelbase/std::max(1.,speed*speed)));
 }
 double step(double steer_key,double throttle_key,double brake_key,double speed,double wheelbase,double dt,const KeyboardParameters& p){
  steer=approach(steer,steer_key,std::abs(steer_key)>std::abs(steer)?p.rise:p.release,dt);
  throttle=approach(throttle,throttle_key,throttle_key>throttle?p.throttle_rise:p.throttle_release,dt);
  brake=approach(brake,brake_key,brake_key>brake?p.brake_rise:p.brake_release,dt);
  return steer*angle_limit(speed,wheelbase,p);
 }
};
