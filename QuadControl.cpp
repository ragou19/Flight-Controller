#include "Common.h"
#include "QuadControl.h"

#include "Utility/SimpleConfig.h"

#include "Utility/StringUtils.h"
#include "Trajectory.h"
#include "BaseController.h"
#include "Math/Mat3x3F.h"
#include <iostream>

#ifdef __PX4_NUTTX
#include <systemlib/param/param.h>
#endif

void QuadControl::Init()
{
  BaseController::Init();

  // variables needed for integral control
  integratedAltitudeError = 0;
    
#ifndef __PX4_NUTTX
  // Load params from simulator parameter system
  ParamsHandle config = SimpleConfig::GetInstance();
   
  // Load parameters (default to 0)
  kpPosXY = config->Get(_config+".kpPosXY", 0);
  kpPosZ = config->Get(_config + ".kpPosZ", 0);
  KiPosZ = config->Get(_config + ".KiPosZ", 0);
     
  kpVelXY = config->Get(_config + ".kpVelXY", 0);
  kpVelZ = config->Get(_config + ".kpVelZ", 0);

  kpBank = config->Get(_config + ".kpBank", 0);
  kpYaw = config->Get(_config + ".kpYaw", 0);

  kpPQR = config->Get(_config + ".kpPQR", V3F());

  maxDescentRate = config->Get(_config + ".maxDescentRate", 100);
  maxAscentRate = config->Get(_config + ".maxAscentRate", 100);
  maxSpeedXY = config->Get(_config + ".maxSpeedXY", 100);

  maxAccelXY = config->Get(_config + ".maxHorizAccel", 100);

  maxTiltAngle = config->Get(_config + ".maxTiltAngle", 100);

  minMotorThrust = config->Get(_config + ".minMotorThrust", 0);
  maxMotorThrust = config->Get(_config + ".maxMotorThrust", 100);
#else
  // load params from PX4 parameter system
  param_get(param_find("MC_PITCH_P"), &Kp_bank);
  param_get(param_find("MC_YAW_P"), &Kp_yaw);
#endif
}

VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
  // Convert a desired 3-axis moment and collective thrust command to 
  //   individual motor thrust commands
  // INPUTS: 
  //   collThrustCmd: desired collective thrust [N]
  //   momentCmd: desired rotation moment about each axis [N m]
  // OUTPUT:
  //   set class member variable cmd (class variable for graphing) where
  //   cmd.desiredThrustsN[0..3]: motor commands, in [N]
	
	// Set rotor-to-rotor distance
	float l = L / std::pow(2.f, 0.5f);

	// Calculate rotational moments
	float tx = momentCmd.x / l;
	float ty = momentCmd.y / l;
	float tz = - momentCmd.z / kappa;
	float F = collThrustCmd;
	float k = kappa;
	
	// Individual thrusts commanded to each rotor
	cmd.desiredThrustsN[0] = (1.f / 4.f) * ( tx + ty + tz + F);
	cmd.desiredThrustsN[1] = (1.f / 4.f) * (-tx + ty - tz + F);
	cmd.desiredThrustsN[3] = (1.f / 4.f) * (-tx - ty + tz + F);
	cmd.desiredThrustsN[2] = (1.f / 4.f) * ( tx - ty - tz + F);

	
	//std::cout << "Thrusts commanded: (";
	//for (int i = 0; i < 3; i++)
	//	std::cout << cmd.desiredThrustsN[i] << ",";
	//std::cout << cmd.desiredThrustsN[3] << ")" << std::endl;

  return cmd;
}

V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
  // Calculate a desired 3-axis moment given a desired and current body rate
  // INPUTS: 
  //   pqrCmd: desired body rates [rad/s]
  //   pqr: current or estimated body rates [rad/s]
  // OUTPUT:
  //   return a V3F containing the desired moments for each of the 3 axes

  V3F momentCmd;

  // Vectorize moments of inertia
  V3F I(Ixx, Iyy, Izz);

  /// Calculate commanded moments
  momentCmd = I * kpPQR * (pqrCmd - pqr);

  return momentCmd;
}

// returns a desired roll and pitch rate 
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
  // Calculate a desired pitch and roll angle rates based on a desired global
  //   lateral acceleration, the current attitude of the quad, and desired
  //   collective thrust command
  // INPUTS: 
  //   accelCmd: desired acceleration in global XY coordinates [m/s2]
  //   attitude: current or estimated attitude of the vehicle
  //   collThrustCmd: desired collective thrust of the quad [N]
  // OUTPUT:
  //   return a V3F containing the desired pitch and roll rates. The Z
  //     element of the V3F should be left at its default value (0)

  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  
  // Set P controller constants
  float xdd = accelCmd[0];
  float ydd = accelCmd[1];
  float c = - collThrustCmd / mass;
  float kpr = kpBank;
  float kpp = kpBank;
  float bxc = 0.0;
  float byc = 0.0;

  // Limit rotation to desired tilt angle range
  if (collThrustCmd > 0.0)
  {
	  bxc = CONSTRAIN(xdd / c, -maxTiltAngle, maxTiltAngle);
	  byc = CONSTRAIN(ydd / c, -maxTiltAngle, maxTiltAngle);
  }

  // Calculate p and q commanded
  float bxcd = kpr * (bxc - R(0,2)); // bxc_dot
  float bycd = kpp * (byc - R(1,2));
  float pc = (1.f / R(2,2)) * ((R(1,0) * bxcd) - (R(0,0) * bycd));
  float qc = (1.f / R(2,2)) * ((R(1,1) * bxcd) - (R(0,1) * bycd));
  
  V3F pqrCmd(pc, qc, 0);

  return pqrCmd;
}

float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
{
  // Calculate desired quad thrust based on altitude setpoint, actual altitude,
  //   vertical velocity setpoint, actual vertical velocity, and a vertical 
  //   acceleration feed-forward command
  // INPUTS: 
  //   posZCmd, velZCmd: desired vertical position and velocity in NED [m]
  //   posZ, velZ: current vertical position and velocity in NED [m]
  //   accelZCmd: feed-forward vertical acceleration in NED [m/s2]
  //   dt: the time step of the measurements [seconds]
  // OUTPUT:
  //   return a collective thrust command in [N]

  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  float thrust = 0;

  // Abbreviated constants for ease of use
  float kzp = kpPosZ;
  float kzd = kpVelZ;
  float kzi = KiPosZ;

  // Integral portion of PID controller
  integratedAltitudeError += (posZCmd - posZ) * dt;

  // Vertical thrust PID controller equation
  float u_1_bar = kzp * (posZCmd - posZ) + kzd * (velZCmd - velZ) + accelZCmd + integratedAltitudeError * kzi;

  // Limit thrust to max ascent and descent rates
  if (u_1_bar <= 0 && velZ <= maxDescentRate)
  {
	  thrust = - (u_1_bar - 9.81) * mass / R(2, 2);
  } 
  else if (u_1_bar >= 0 && velZ >= maxAscentRate)
  {
	  thrust = - (u_1_bar - 9.81) * mass / R(2, 2);
  }
  else if (velZ >= maxDescentRate && velZ <= maxAscentRate)
  {
	  thrust = - (u_1_bar - 9.81) * mass / R(2, 2);
  }
  
  return thrust;
}

// returns a desired acceleration in global frame
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmdFF)
{
  // Calculate a desired horizontal acceleration based on 
  //  desired lateral position/velocity/acceleration and current pose
  // INPUTS: 
  //   posCmd: desired position, in NED [m]
  //   velCmd: desired velocity, in NED [m/s]
  //   pos: current position, NED [m]
  //   vel: current velocity, NED [m/s]
  //   accelCmdFF: feed-forward acceleration, NED [m/s2]
  // OUTPUT:
  //   return a V3F with desired horizontal accelerations. 
  //     the Z component should be 0

  // make sure we don't have any incoming z-component
  accelCmdFF.z = 0;
  velCmd.z = 0;
  posCmd.z = pos.z;

  // we initialize the returned desired acceleration to the feed-forward value.
  V3F accelCmd = accelCmdFF;

  // Limit velocity to specified maximums
  velCmd.constrain(-maxSpeedXY, maxSpeedXY);

  // Update feed forward with local position and velocity
  accelCmd += kpPosXY * (posCmd - pos) + kpVelXY * (velCmd - vel);

  // Limit commanded acceleration
  accelCmd.constrain(-maxAccelXY, maxAccelXY);

  // Reset z component to 0
  accelCmd.z = 0;

  return accelCmd;
}

// returns desired yaw rate
float QuadControl::YawControl(float yawCmd, float yaw)
{
  // Calculate a desired yaw rate to control yaw to yawCmd
  // INPUTS: 
  //   yawCmd: commanded yaw [rad]
  //   yaw: current yaw [rad]
  // OUTPUT:
  //   return a desired yaw rate [rad/s]

  float yawRateCmd=0;
  
  // Limit commanded yaw to [0,2*pi]
  yawCmd = fmodf(yawCmd, 2.0 * F_PI);

  // Calculate yaw error and set to standard position of the angle
  float yawError = yawCmd - yaw;

  if (yawError > F_PI)
  {
	  yawError -= 2.0 * F_PI;
  }
  else if (yawError < -F_PI)
  {
	  yawError += 2.0 * F_PI;
  }

  // P controller for yaw
  yawRateCmd = kpYaw * (yawError);

  return yawRateCmd;

}

VehicleCommand QuadControl::RunControl(float dt, float simTime)
{
  curTrajPoint = GetNextTrajectoryPoint(simTime);

  float collThrustCmd = AltitudeControl(curTrajPoint.position.z, curTrajPoint.velocity.z, estPos.z, estVel.z, estAtt, curTrajPoint.accel.z, dt);

  // reserve some thrust margin for angle control
  float thrustMargin = .1f*(maxMotorThrust - minMotorThrust);
  collThrustCmd = CONSTRAIN(collThrustCmd, (minMotorThrust+ thrustMargin)*4.f, (maxMotorThrust-thrustMargin)*4.f);
  
  V3F desAcc = LateralPositionControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel, curTrajPoint.accel);
  
  V3F desOmega = RollPitchControl(desAcc, estAtt, collThrustCmd);
  desOmega.z = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw());

  V3F desMoment = BodyRateControl(desOmega, estOmega);

  return GenerateMotorCommands(collThrustCmd, desMoment);
}
