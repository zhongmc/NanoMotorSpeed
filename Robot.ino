#include <Arduino.h>


double max_vel, min_vel;
double r, l;
int maxRPM, minRPM;

double m_per_tick_l, m_per_tick_r;
int ticks_per_rev_l, ticks_per_rev_r;

double x, y, theta;
double v, w, vel_l, vel_r; 
long prev_left_ticks, prev_right_ticks;

void initRobot()
{
    x = 0;
    y = 0;
    theta = 0;
    v = 0;
    w = 0;
    // initRobot(0.032627,  0.127, 960, 960, 10, 70); //0.1266
  initRobot(0.031025,  0.1590, 990, 990, 19, 85);

}

void resetRobot()
{
  x = 0;
  y = 0;
  theta = 0;
  v = 0;
  w = 0;
  prev_left_ticks = 0;
  prev_right_ticks = 0;
  count1 = 0;
  count2 = 0;

  
}

void initRobot(double R, double L, double ticksr_l, double ticksr_r, double min_rpm, double max_rpm)
{
  x = 0;
  y = 0;
  theta = 0;
  v = 0;
  w = 0;
  prev_left_ticks = 0;
  prev_right_ticks = 0;
  r = R;
  l = L;
  ticks_per_rev_l = ticksr_l; //20;
  ticks_per_rev_r = ticksr_r; //20;
  m_per_tick_l = 2 * PI * r / ticks_per_rev_l;
  m_per_tick_r = 2 * PI * r / ticks_per_rev_r;
  max_vel = max_rpm * 2 * PI / 60;
  min_vel = min_rpm * 2 * PI / 60;
  prev_left_ticks = 0;
  prev_right_ticks = 0;
  maxRPM = max_rpm;
  minRPM = min_rpm;
}

double getMinVel()
{
    return min_vel;
}

double getMaxVel()
{
    return max_vel;
}

void updateRobotState(long _left_ticks, long _right_ticks, double dt)
{
  //  long left_ticks, right_ticks;
  if (prev_right_ticks == _right_ticks && prev_left_ticks == _left_ticks)
  {
    w = 0;
    v = 0;
    return; //no change
  }

  double d_right, d_left, d_center;

  vel_l = ((double)(_left_ticks - prev_left_ticks) / dt) / (double)ticks_per_rev_l;
  vel_r = ((double)(_right_ticks - prev_right_ticks) / dt) / (double)ticks_per_rev_r;
  vel_l = 2 * PI * vel_l;
  vel_r = 2 * PI * vel_r;

  d_left = (_left_ticks - prev_left_ticks) * m_per_tick_l;
  d_right = (_right_ticks - prev_right_ticks) * m_per_tick_r;

  prev_left_ticks = _left_ticks;
  prev_right_ticks = _right_ticks;

  d_center = (d_right + d_left) / 2;
  v = d_center / dt;

  double phi = (d_right - d_left) / l;

  w = phi / dt;

  x = x + d_center * cos(theta);
  y = y + d_center * sin(theta);
  theta = theta + phi;
  theta = atan2(sin(theta), cos(theta));
}

//用单边减速的方式转弯；uni_to_diff 采用的是一边加，一边减的方式，速度不变，容易冲
Vel uni_to_diff_oneside(double v, double w)
{

  Vel vel;
  double wv = (w*l)/(2*r);
  if( abs(v) <= 0.001 )
  {
    double wvel = wv;
    if( abs(wv) < 1.2*min_vel )
    {
      if( wv < 0 )
        wvel = -1.2*min_vel;
      else
      {
        wvel = 1.2*min_vel;
      }
    }
    
    vel.vel_r =  wvel; //(2 * v + w * l) / (2 * r);
    vel.vel_l = -wvel; // (2 * v - w * l) / (2 * r);
    return vel;
  }

  double vvel = v/r;
  if( v*w >= 0 )
  {
      vel.vel_r = vvel;
      vel.vel_l = vvel - wv*2;
      if( vel.vel_l * v < 0 )
        vel.vel_l = 0;
  }
  else
  {
      vel.vel_l = vvel;
      vel.vel_r = vvel + wv*2;
      if( vel.vel_r * v < 0 )
        vel.vel_r = 0;
  }

  return vel;

}


  Vel uni_to_diff_v(double v, double w )
  {
    Vel vel;

    double wv =  ( w * l) / (2 * r);
    if( abs(v) <= 0.001 )
    {
      double wvel = wv;
      if( abs(wv) < 1.2*min_vel )
      {
        if( wv < 0 )
          wvel = -1.2*min_vel;
        else
        {
          wvel = 1.2*min_vel;
        }
      }
      
      vel.vel_r =  wvel; //(2 * v + w * l) / (2 * r);
      vel.vel_l = -wvel; // (2 * v - w * l) / (2 * r);
      return vel;
    }

    double vvel = v/r;
    if( abs(vvel) < 1.2*min_vel )
    {
        if( v < 0 )
          vvel =  -1.2*min_vel;
        else
        {
          vvel =  1.2*min_vel;
        }
         
    }
    vel.vel_l = vvel;
    vel.vel_r = vvel;

    if( w < 0 )
    {
      if( v < 0 )
      {
          vel.vel_l = vel.vel_l - wv;
          if( vel.vel_l > 0 )
            vel.vel_l = 0;
      }
      else
      {
          vel.vel_r = vel.vel_r + wv;
          if( vel.vel_r < 0 )
            vel.vel_r = 0;
      }      
    }
    else
    {
      if( v < 0 )
      {
          vel.vel_r = vel.vel_r + wv;
          if( vel.vel_r > 0 )
            vel.vel_r = 0;
      }
      else
      {
          vel.vel_l = vel.vel_l - wv;
          if( vel.vel_l < 0 )
            vel.vel_l = 0;
      }      
    }
    return vel;
  }

double vel_to_pwm(double vel)
{
  double nvel = abs(vel);
  if( nvel < min_vel )
    return 0;
  if( nvel > max_vel )
    nvel = max_vel;
  double pwm = 20 * nvel + 12;  //22.6 13.6
  if (vel < 0)
    return -pwm;
  return pwm;
}


double pwm_to_ticks_l(double pwm, double dt)
{
  if (pwm == 0)
    return 0;

  double npwm = abs(pwm);
  double ticks = dt * (0.0091*npwm*npwm + 5.8203*npwm - 60.155);
  // double ticks = dt * (7.3927 * npwm - 117.23);
  if (pwm > 0)
    return ticks;
  else
    return -ticks;
}

double pwm_to_ticks_r(double pwm, double dt)
{
  if (pwm == 0)
    return 0;

  double npwm = abs(pwm);
  double ticks = dt * (-0.0015*npwm*npwm + 6.2062*npwm - 63.505);
  // double ticks = dt * (5.9839 * npwm - 56.46);
  if (pwm > 0)
    return ticks;
  else
    return -ticks;
}
