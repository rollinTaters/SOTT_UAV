/*
	MIT License

	Copyright (c) 2025 rollinTaters

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:
	
	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.
	
	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.
*/


#include "utility.hpp"

#include <iostream> // DEBUG, OP ostream

// ---- Point ----
Point::Point():x(0), y(0), z(0) {}
Point::Point( const float in_x, const float in_y, const float in_z ):
    x(in_x), y(in_y), z(in_z) {}

float Point::mag() const
{
    return std::hypot(x,y,z);
}

Point Point::unit() const
{
    float m = mag();
    return Point( x/m, y/m, z/m );
}

float Point::absDist( const Point inp ) const
{
    Point delta = *this - inp;
    return delta.mag();
}

float Point::heading() const
{
    // start by making a unit vector
    Point unit_vec(x,y,0);
    unit_vec = unit_vec.unit();

    float rad = 0;

    // this whole thing is a shit-show.
    // avert your eyes for your own good.
    // you have been warned.

    // on axis (and possible divide by zero) checks
    if( unit_vec.x == 0 )
    {
        if( unit_vec.y > 0 )
            return 0;   // north 000
        else
            return PI;  // south 180
    }
    if( unit_vec.y == 0 )
    {
        if( unit_vec. x > 0 )
            return PI/2.f;  // east 090
        else
            return (3.f/2.f) * PI; // west 270
    }
    // an edge case where the input vector had only z component (which we discarded)
    // or no components at all
    if( unit_vec.x == 0 && unit_vec.y == 0 ) return 0;

    // use arcsin when closer to x axis
    bool use_arcsin = false;
    if( abs(unit_vec.x) > abs(unit_vec.y) )
        use_arcsin = true;

    // here be demons, turn back
    if( unit_vec.y > 0 )
    {
        if( unit_vec.x > 0 )
        {
            // sector I
            if( use_arcsin )
                rad = asin( unit_vec.y );
            else
                rad = (PI/2.f) - acos( unit_vec.x );
            //rad = PI/2.f - rad;
        }else{
            // sector II
            if( use_arcsin )
                rad = asin( unit_vec.y );
            else
                rad = (PI/2.f) - acos( -unit_vec.x );
            rad = /*(3.f*PI)/2.f*/ 2*PI - rad;
        }
    }else{
        if( unit_vec.x > 0 )
        {
            // sector IV
            if( use_arcsin )
                rad = asin( -unit_vec.y );
            else
                rad = (PI/2.f) - acos( unit_vec.x );
            rad = PI/*/2.f*/ - rad;
        }else{
            // sector III
            if( use_arcsin )
                rad = asin( -unit_vec.y );
            else
                rad = (PI/2.f) - acos( -unit_vec.x );
            rad = PI + rad;
        }
    }

    // fucking finally
    return rad;
}

float Point::sqErrSep( const Point p1, float seperation )
{
    Point delta = *this - p1;
    return std::pow( seperation - delta.mag(), 2 );
}

Point& Point::operator+=( const Point &rhs )
{
    x += rhs.x;
    y += rhs.y;
    z += rhs.z;
    return *this;
}

Point& Point::operator-=( const Point &rhs )
{
    x -= rhs.x;
    y -= rhs.y;
    z -= rhs.z;
    return *this;
}

Point& Point::operator*=( const Point &rhs )
{
    x *= rhs.x;
    y *= rhs.y;
    z *= rhs.z;
    return *this;
}

Point& Point::operator*=( const float & f )
{
    x*=f;
    y*=f;
    z*=f;
    return *this;
}

Point& Point::operator/=( const float & f )
{
    x/=f;
    y/=f;
    z/=f;
    return *this;
}

bool Point::operator==( const Point& p ) const
{
    return (x==p.x) && (y==p.y) && (z==p.z);
}

Point operator+( const Point &p1, const Point &p2 )
{
    return Point( p1.x+p2.x, p1.y+p2.y, p1.z+p2.z );
}

Point operator-( const Point &p1, const Point &p2 )
{
    return Point( p1.x-p2.x, p1.y-p2.y, p1.z-p2.z );
}

Point operator*( const Point &p1, const float f )
{
    return Point( p1.x*f, p1.y*f, p1.z*f );
}

Point operator/( const Point &p1, const float f )
{
    return Point( p1.x/f, p1.y/f, p1.z/f );
}

// ---- End of Point ----

cQuaternion &cQuaternion::operator=(const cQuaternion &q)
{
	w = q.w;
	x = q.x;
	y = q.y;
	z = q.z;
	return *this;
}

cQuaternion &cQuaternion::operator*=(const cQuaternion &q)
{
	float w1 = w;
	float x1 = x;
	float y1 = y;
	float z1 = z;

	float w2 = q.w;
	float x2 = q.x;
	float y2 = q.y;
	float z2 = q.z;

	w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
	x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
	y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2;
	z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2;
	return *this;
}

float cQuaternion::dot(const cQuaternion &q) const
{
	return w * q.w + x * q.x + y * q.y + z * q.z;
}

float cQuaternion::norm() const
{
	return sqrtf(w * w + x * x + y * y + z * z);
}

cQuaternion &cQuaternion::normalize()
{
	float iLen = 1 / norm();
	w *= iLen;
	x *= iLen;
	y *= iLen;
	z *= iLen;
	return *this;
}

const cQuaternion cQuaternion::conjugate() const
{
	return cQuaternion(w, -x, -y, -z);
}

void cQuaternion::rotateVector( v3f &vec ) const
{
	// t = 2q x v
	float tx = 2. * (y * vec.z - z * vec.y);
	float ty = 2. * (z * vec.x - x * vec.z);
	float tz = 2. * (x * vec.y - y * vec.x);

	// v + w t + q x t
	vec.x = vec.x + w * tx + y * tz - z * ty;
	vec.y = vec.y + w * ty + z * tx - x * tz;
	vec.z = vec.z + w * tz + x * ty - y * tx;
}

const cQuaternion cQuaternion::fromEuler( v3f euler ) 
{
	float x = euler.x * 0.5;
	float y = euler.y * 0.5;
	float z = euler.z * 0.5;

	float cX = cosf(x);
	float cY = cosf(y);
	float cZ = cosf(z);

	float sX = sinf(x);
	float sY = sinf(y);
	float sZ = sinf(z);

#define QUATERNION_EULER_ORDER QUATERNION_EULER_ZXY
//#define QUATERNION_EULER_ORDER QUATERNION_EULER_XYZ // roll around X, pitch around Y, yaw around Z
//#define QUATERNION_EULER_ORDER QUATERNION_EULER_YXZ // deviceorientation
//#define QUATERNION_EULER_ORDER QUATERNION_EULER_ZYX
//#define QUATERNION_EULER_ORDER QUATERNION_EULER_YZX
//#define QUATERNION_EULER_ORDER QUATERNION_EULER_XZY

#if QUATERNION_EULER_ORDER == QUATERNION_EULER_ZXY
	// axisAngle([0, 0, 1], φ) * axisAngle([1, 0, 0], θ) * axisAngle([0, 1, 0], ψ)
	return cQuaternion(
		cX * cY * cZ - sX * sY * sZ,
		sY * cX * cZ - sX * sZ * cY,
		sX * sY * cZ + sZ * cX * cY,
		sX * cY * cZ + sY * sZ * cX);
#elif QUATERNION_EULER_ORDER == QUATERNION_EULER_XYZ // roll around X, pitch around Y, yaw around Z
	// axisAngle([1, 0, 0], φ) * axisAngle([0, 1, 0], θ) * axisAngle([0, 0, 1], ψ)
	return cQuaternion(
		cX * cY * cZ - sX * sY * sZ,
		sX * cY * cZ + sY * sZ * cX,
		sY * cX * cZ - sX * sZ * cY,
		sX * sY * cZ + sZ * cX * cY);
#elif QUATERNION_EULER_ORDER == QUATERNION_EULER_YXZ // deviceorientation
	// axisAngle([0, 1, 0], φ) * axisAngle([1, 0, 0], θ) * axisAngle([0, 0, 1], ψ)
	return cQuaternion(
		sX * sY * sZ + cX * cY * cZ,
		sX * sZ * cY + sY * cX * cZ,
		sX * cY * cZ - sY * sZ * cX,
		sZ * cX * cY - sX * sY * cZ);
#elif QUATERNION_EULER_ORDER == QUATERNION_EULER_ZYX
	// axisAngle([0, 0, 1], φ) * axisAngle([0, 1, 0], θ) * axisAngle([1, 0, 0], ψ)
	return cQuaternion(
		sX * sY * sZ + cX * cY * cZ,
		sZ * cX * cY - sX * sY * cZ,
		sX * sZ * cY + sY * cX * cZ,
		sX * cY * cZ - sY * sZ * cX);
#elif QUATERNION_EULER_ORDER == QUATERNION_EULER_YZX
	// axisAngle([0, 1, 0], φ) * axisAngle([0, 0, 1], θ) * axisAngle([1, 0, 0], ψ)
	return cQuaternion(
		cX * cY * cZ - sX * sY * sZ,
		sX * sY * cZ + sZ * cX * cY,
		sX * cY * cZ + sY * sZ * cX,
		sY * cX * cZ - sX * sZ * cY);
#elif QUATERNION_EULER_ORDER == QUATERNION_EULER_XZY
	// axisAngle([1, 0, 0], φ) * axisAngle([0, 0, 1], θ) * axisAngle([0, 1, 0], ψ)
	return cQuaternion(
		sX * sY * sZ + cX * cY * cZ,
		sX * cY * cZ - sY * sZ * cX,
		sZ * cX * cY - sX * sY * cZ,
		sX * sZ * cY + sY * cX * cZ);
#elif QUATERNION_EULER_ORDER == QUATERNION_EULER_ZYZ
	// axisAngle([0, 0, 1], φ) * axisAngle([0, 1, 0], θ) * axisAngle([0, 0, 1], ψ)
	return cQuaternion(
		cX * cY * cZ - sX * sZ * cY,
		sY * sZ * cX - sX * sY * cZ,
		sX * sY * sZ + sY * cX * cZ,
		sX * cY * cZ + sZ * cX * cY);
#elif QUATERNION_EULER_ORDER == QUATERNION_EULER_ZXZ
	// axisAngle([0, 0, 1], φ) * axisAngle([1, 0, 0], θ) * axisAngle([0, 0, 1], ψ)
	return cQuaternion(
		cX * cY * cZ - sX * sZ * cY,
		sX * sY * sZ + sY * cX * cZ,
		sX * sY * cZ - sY * sZ * cX,
		sX * cY * cZ + sZ * cX * cY);
#elif QUATERNION_EULER_ORDER == QUATERNION_EULER_YXY
	// axisAngle([0, 1, 0], φ) * axisAngle([1, 0, 0], θ) * axisAngle([0, 1, 0], ψ)
	return cQuaternion(
		cX * cY * cZ - sX * sZ * cY,
		sX * sY * sZ + sY * cX * cZ,
		sX * cY * cZ + sZ * cX * cY,
		sY * sZ * cX - sX * sY * cZ);
#elif QUATERNION_EULER_ORDER == QUATERNION_EULER_YZY
	// axisAngle([0, 1, 0], φ) * axisAngle([0, 0, 1], θ) * axisAngle([0, 1, 0], ψ)
	return cQuaternion(
		cX * cY * cZ - sX * sZ * cY,
		sX * sY * cZ - sY * sZ * cX,
		sX * cY * cZ + sZ * cX * cY,
		sX * sY * sZ + sY * cX * cZ);
#elif QUATERNION_EULER_ORDER == QUATERNION_EULER_XYX
	// axisAngle([1, 0, 0], φ) * axisAngle([0, 1, 0], θ) * axisAngle([1, 0, 0], ψ)
	return cQuaternion(
		cX * cY * cZ - sX * sZ * cY,
		sX * cY * cZ + sZ * cX * cY,
		sX * sY * sZ + sY * cX * cZ,
		sX * sY * cZ - sY * sZ * cX);
#elif QUATERNION_EULER_ORDER == QUATERNION_EULER_XZX
	// axisAngle([1, 0, 0], φ) * axisAngle([0, 0, 1], θ) * axisAngle([1, 0, 0], ψ)
	return cQuaternion(
		cX * cY * cZ - sX * sZ * cY,
		sX * cY * cZ + sZ * cX * cY,
		sY * sZ * cX - sX * sY * cZ,
		sX * sY * sZ + sY * cX * cZ);
#endif
}

const cQuaternion cQuaternion::fromAxisAngle( v3f axis, float radian ) 
{
	cQuaternion ret;

	float halfAngle = radian * 0.5;

	float sin_2 = sinf(halfAngle);
	float cos_2 = cosf(halfAngle);

	float sin_norm = sin_2 / sqrtf( axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);

	ret.w = cos_2;
	ret.x = axis.x * sin_norm;
	ret.y = axis.y * sin_norm;
	ret.z = axis.z * sin_norm;

	return ret;
}

std::ostream& operator<<( std::ostream& os,  const OP& op )
{
    os<< "pos: "
      << op.pos.x <<"x "
      << op.pos.y <<"y "
      << op.pos.z <<"z  mag: "
      << op.pos.mag() <<"\n";
    os<< "att: "
      << op.att.x <<"x "
      << op.att.y <<"y "
      << op.att.z <<"z "
      << op.att.w <<"w  mag: "
      << op.att.norm() <<"\n";
    return os;
}

std::ostream& operator<<( std::ostream& os, const v3f& p )
{
    os<< p.x <<"x "<< p.y <<"y "<< p.z <<"z";
    return os;
}


BB3D::BB3D()
{
    m_pos = v3f(0,0,0);
    m_size = v3f(1,1,1);
}

BB3D::BB3D( const v3f pos,
            const cQuaternion quat,
            const v3f size ):
    m_pos(pos),m_quat(quat),m_size(size)
{
}

void BB3D::yawLeft( float radian )
{
    v3f axis = getLocalVecZ();
    // create a quat that rotates us in local csys
    cQuaternion q = cQuaternion::fromAxisAngle( axis, radian );

    // apply quat to our member quat
    m_quat *= q;
}

void BB3D::pitchUp( float radian )
{
    v3f axis = getLocalVecX();
    // create a quat that rotates us in local csys
    cQuaternion q = cQuaternion::fromAxisAngle( axis, radian );

    // apply quat to our member quat
    m_quat *= q;
}

void BB3D::rollRight( float radian )
{
    v3f axis = getLocalVecY();
    // create a quat that rotates us in local csys
    cQuaternion q = cQuaternion::fromAxisAngle( axis, radian );

    // apply quat to our member quat
    m_quat *= q;
}

void BB3D::translateLocal( const v3f t )
{
    m_pos += getLocalVecX() * t.x +
             getLocalVecY() * t.y +
             getLocalVecZ() * t.z;
}
void BB3D::translateFWD( const float meter )   { translateLocal(v3f(meter,0,0)); }
void BB3D::translateRight( const float meter ) { translateLocal(v3f(0,meter,0)); }
void BB3D::translateUp( const float meter )    { translateLocal(v3f(0,0,meter)); }

// -- Setters --
void BB3D::setOP( const OP op )
{
    m_pos = op.pos;
    m_quat = op.att;
}

void BB3D::setSize( const v3f size )
{
    m_size = size;
}

void BB3D::setSize( const float x, const float y, const float z )
{
    m_size = v3f(x,y,z);
}

void BB3D::setPos( const v3f pos )
{
    m_pos = pos;
}

void BB3D::setPos( const float x, const float y, const float z )
{
    m_pos = v3f(x,y,z);
}

void BB3D::setAng( const v3f euler )
{
    m_quat = cQuaternion::fromEuler( euler );
}

void BB3D::setAng( const float x, const float y, const float z )
{
    setAng( v3f(x,y,z) );
}

void BB3D::setAtt( const cQuaternion q ) { m_quat = q; }

// -- Getters --
OP BB3D::getOP() const { return OP{ m_pos, m_quat }; }

v3f BB3D::getSize() const { return m_size; }

v3f BB3D::getPos() const { return m_pos; }

v3f BB3D::getAngEuler() const
{
    // conversion yanked from, then modified:
    // https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/indexLocal.htm

    // TODO axis naming convention must be checked
    cQuaternion q1 = m_quat;
	double sqw = q1.w*q1.w;
    double sqx = q1.x*q1.x;
    double sqy = q1.y*q1.y;
    double sqz = q1.z*q1.z;
    float heading = std::atan2(2.0 * (q1.x*q1.y + q1.z*q1.w),(sqx - sqy - sqz + sqw));
    float bank = std::atan2(2.0 * (q1.y*q1.z + q1.x*q1.w),(-sqx - sqy + sqz + sqw));
    float attitude = std::asin(-2.0 * (q1.x*q1.z - q1.y*q1.w));

    return v3f( attitude, bank, heading );
}

cQuaternion BB3D::getQuaternion() const { return m_quat; }

v3f BB3D::getLocalVecX() const
{
    v3f ret(1,0,0);
    m_quat.rotateVector( ret );
    return ret;
}

v3f BB3D::getLocalVecY() const 
{
    v3f ret(0,1,0);
    m_quat.rotateVector( ret );
    return ret;
}

v3f BB3D::getLocalVecZ() const
{
    v3f ret(0,0,1);
    m_quat.rotateVector( ret );
    return ret;
}

BB3D BB3D::onTop( BB3D base ) const
{
    // make a copy to modify and return
    BB3D copy = BB3D(*this);

    // -- translations --
    // move this by this objects position amount (which is defined relative to base)
    // on base objects local csys vectors
    base.translateLocal( m_pos );
    copy.m_pos = base.m_pos;

    // FIXME this shit be broken
    // -- rotations --
    /*
    v3f q_vec = { m_quat.x, m_quat.y, m_quat.z };
    base.m_quat.rotateVector( q_vec );
    copy.m_quat.x = q_vec.x;
    copy.m_quat.y = q_vec.y;
    copy.m_quat.z = q_vec.z;
    */
    copy.m_quat = base.m_quat * copy.m_quat;

    return copy;
}


// adds positions and angles, uses left operands size
BB3D operator +( BB3D left, BB3D right )
{
    BB3D ret = left;
    ret.m_pos += right.m_pos;
    ret.m_quat *= right.m_quat;

    return ret;
}

// adds positions and angles, uses left operands size
BB3D& operator +=( BB3D& left, BB3D right )
{
    BB3D ret = left;
    ret.m_pos += right.m_pos;
    ret.m_quat *= right.m_quat;

    left = ret;
    return left;
}
