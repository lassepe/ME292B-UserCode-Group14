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

  namespace World {
    const float gravity = 9.81f;  // acceleration of gravity [m/s^2]
  };
}
