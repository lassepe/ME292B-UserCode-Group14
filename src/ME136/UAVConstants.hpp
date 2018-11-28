#pragma once

//Some constants that we may use
namespace Constants {
  namespace UAV {
    const float mass = 32e-3f;  // mass of the quadcopter [kg]
    const float propOrthDist = 33e-3f; // the orthogonal distance of the propeller to each axis [m]
    const float kappa = 0.01; // the proportional constant from thrust force to torque [m]
    const float inertia_xx = 16e-6f;  //MMOI about x axis [kg.m^2]
    const float inertia_yy = inertia_xx;  //MMOI about y axis [kg.m^2]
    const float inertia_zz = 29e-6f;  //MMOI about z axis [kg.m^2]
    const float dt = 1.0f / 500.0f; //[s] period between successive calls to MainLoop
  };

  namespace StateEstimation {
    const float rhoAttitude = 0.01f;
    const float rhoHorizontalVel = 0.1f;
    const float rhoVertical = 0.3f;
  };

  namespace Control {
    //Gyro rates
    const float timeConstant_rollRate = 0.04f;
    const float timeConstant_pitchRate = timeConstant_rollRate;
    const float timeConstant_yawRate = 0.1f;
    //Angles
    const float timeConstant_rollAngle = 0.12f;
    const float timeConstant_pitchAngle = timeConstant_rollAngle;
    const float timeConstant_yawAngle = 0.2f;
    // PID poisition controller
    const float pPos = 1.5f;
    const float iPos = 2.f;
    const float dPos = 3.f;
    // verticl control
    const float natFreq_height = 3.f;
    const float dampRat_height = 1.f;
  };

  namespace World {
    const float gravity = 9.81f;  // acceleration of gravity [m/s^2]
  };
}
