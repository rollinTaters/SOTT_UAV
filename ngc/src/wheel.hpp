/*

	MIT License

	Copyright (c) 2025 rollinTaters


    Uncessary and over-engineered tyre deflection and contact patch force simulations for wheels

    NOTE: To be used by Environment Emulator class for simuations. DO NOT add into vehicle class
*/

#include "utility.hpp"

struct ContactPatch
{
    bool isActive = false;    // is the patch actually touching something
    v3f normal; // normal vector
    v3f force;  // external forces applied to the patch, (weight of car, torque of wheel)
    v3f pos;    // position, relative to the wheel itself
    //v3f slip;   // velocity of slip (m/s)
    float coef_static_friction = 0.7f;
    float coef_dynamic_friction = 0.5f;

    // returns the (dynamic or static) friction forces, no normal forces
    v3f getFrictionForce();
};

class Wheel
{
  public:
    Wheel( v3f pos, v3f axis, v3f steer_axis, float dia, float thickness );

    // sets the axis of rotation of wheel relative to its nominal axis of rotation
    void setSteerAngle( float radian );
    void setTorque( float torque ); // apply torque to wheel (Nm)

    v3f getReactionForce(); // returns calculated reaction force from contact patches
    v3f getAxis() const;

    // TODO determine contact patch collisions with ground mesh
    // TODO populate contact patch forces

  private:
    v3f m_pos;
    v3f m_axis;
    v3f m_steer_axis;
    v3f m_nominal_axis;
    const float m_dia;
    const float m_thickness;

    float m_torque;
    ContactPatch[3] m_patches;

};
