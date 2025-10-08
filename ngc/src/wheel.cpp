#include "wheel.cpp"

v3f ContactPatch::getFrictionForce()
{
    if( !isActive ) return {0,0,0};

    v3f normal_force = normal * force;
    v3f friction_force = force - normal_force;

    float max_static_fric = normal_force.mag() * coef_static_friction;
    float max_dynamic_fric = normal_force.mag() * coef_dynamic_friction;

    if( friction_force.mag() > max_static_fric )
    {
        // switch to dynamic friction
        // get the unit vector
        friction_force = friction_force.unit();

        // scale it by max dynamic friction force
        friction_force = friction_force * max_dynamic_fric;

        // update total force
        force = normal_force + friction_force;
    }else{
        // in static friction regime, all good
    }

    return friction_force;
}

Wheel::Wheel( v3f pos, v3f axis, v3f steer_axis, float dia, float thickness ):
    m_pos(pos), m_nominal_axis(axis), m_axis(axis), m_steer_axis(steer_axis), m_dia(dia), m_thickness(thickness)
{
    // init contact patches
    for( int i = 0; i < 3; i++ )
    {
        // to rotate(offset) each patch to its position on wheel surface
        cQuaternion q = cQuaternion::FromAxisAngle( m_nominal_axis, (30*(PI/180)) - (i*30*PI/180) );
        m_patches[i].pos = /*m_pos -*/ v3f(0,0,-m_dia/2);
        q.rotateVector( m_patches[0].pos );

        // calculate normal vectors
        m_patches[i].normal = -m_patches.pos;
        m_patches[i].normal = m_patches[i].normal.norm();
    }

    m_torque = 0;
}

void Wheel::setSteerAngle( float radian )
{
    v3f new_axis = m_nominal_axis;
    cQuaternion q = cQuaternion::FromAxisAngle( m_steer_axis, radian );
    q.rotateVector( new_axis );
    m_axis = new_axis;
}

void Wheel::setTorque( float torque ) { m_torque = torque; }

v3f Wheel::getReactionForce()
{
    // combine all reaction forces from contact patches and return them
    return m_patches[0].force + m_patches[1].force + m_patches[2].force;
}

v3f Wheel::getAxis() const { return m_axis; }
