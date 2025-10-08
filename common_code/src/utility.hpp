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


/*
   Some utility functions to make life easier

*/

#pragma once

#include <cmath>
#include <iostream>     // DEBUG

#ifndef PI
#define PI 3.141592
#endif


struct Point
{
    float x;
    float y;
    float z;

    Point();
    Point( const float, const float, const float );

    float mag() const;  // returns magnitude of this point (threats it as a vector)
    Point unit() const; // returns unit vector of this point (threats it as a vector)
    float absDist( const Point ) const; // returns absolute distance between this and given
    float heading() const;  // north is +Y, Z (up-down) is ignored, return value is in radians

    // returns squared error of desired seperation between two points
    float sqErrSep( const Point p1, float seperation );

    Point& operator+=( const Point & );
    Point& operator-=( const Point & );
    Point& operator*=( const Point & );
    Point& operator*=( const float & );
    Point& operator/=( const float & );
    bool operator==(const Point & ) const;

};

typedef Point v3f;

Point operator+( const Point &, const Point & );
Point operator-( const Point &, const Point & );
Point operator*( const Point &, const float );
Point operator/( const Point &, const float );

class cQuaternion
{
  public:
    float w;
    float x;
    float y;
    float z;

    cQuaternion() : w(1), x(0), y(0), z(0) {}
    cQuaternion( const cQuaternion &q ) : w(q.w), x(q.x), y(q.y), z(q.z) {}
    cQuaternion( float _x, float _y, float _z ) : w(0), x(_x), y(_y), z(_z) {}
    cQuaternion( float _w, float _x, float _y, float _z ) : w(_w), x(_x), y(_y), z(_z) {}

    cQuaternion &operator=( const cQuaternion &rhs );
    cQuaternion &operator*=( const cQuaternion &q );
    const cQuaternion operator*( const cQuaternion &q ) const { return cQuaternion(*this) *= q; }
    const cQuaternion operator*( const float fl ) const { return cQuaternion(w*fl,x*fl,y*fl,z*fl); }

    float dot( const cQuaternion &q ) const;
    float norm() const;
    cQuaternion &normalize();

    const cQuaternion conjugate() const;
    void rotateVector( v3f &vec ) const;

    // these create a new quaternion and return it
    static const cQuaternion fromEuler( v3f euler );
    static const cQuaternion fromAxisAngle( v3f axis, float radian );
};

// Oriented Point
struct OP
{
    v3f pos{0,0,0};             // position
    cQuaternion att{0,0,0,0};   // attitude
};

// DEBUG
std::ostream& operator<<( std::ostream& os, const OP& op );
std::ostream& operator<<( std::ostream& os, const v3f& p );


// 3 dimensional bounding box geometry
struct BB3D
{
  public:
    BB3D();
    BB3D( const v3f pos,
          const cQuaternion quat,
          const v3f size );

    // these rotate around the local object axes
    void yawLeft( float radian );
    void pitchUp( float radian );
    void rollRight( float radian );

    // these translate on the local object axes
    void translateLocal( const v3f );
    void translateFWD( const float meter );
    void translateRight( const float meter );
    void translateUp( const float meter );


    // -- setters --
    void setOP( const OP );
    void setSize( const v3f size );
    void setSize( const float x, const float y, const float z );

    void setPos( const v3f pos );
    void setPos( const float x, const float y, const float z );

    void setAng( const v3f euler );    // (radian)
    void setAng( const float x, const float y, const float z ); // (radian)
    void setAtt( const cQuaternion );

    // -- getters --
    OP getOP() const;

    v3f getSize() const;

    v3f getPos() const;

    v3f getAngEuler() const;   // euler angles (in order: z_yaw, x_pitch, y_roll) (radian)
    cQuaternion getQuaternion() const;   // returns quaternion describing the rotation of local csys

    v3f getLocalVecX() const;
    v3f getLocalVecY() const;
    v3f getLocalVecZ() const;

    // assumes that this object has its csys defined relative to the input BB3D and returns a copy of this BB3D object, that has its position and angles made relative to the global csys
    BB3D onTop( BB3D ) const;

  private:
    // unless otherwise specified, all 3d vectors are:
    // +X:starboard, +Y:bow, +Z:above, (or width, len, height) (metre)
    // +X:pitch up, +Y:roll right, +Z:yaw left      (radian)

    // positional variables
    v3f m_pos;      // (metre)
    cQuaternion m_quat;       // a quaternion describing our rotation relative to global

    // size variable
    v3f m_size;     // (metre)

    friend BB3D operator +( BB3D left, BB3D right );
    friend BB3D& operator +=( BB3D& left, BB3D right );
};

// adds positions and angles, uses left operands size
BB3D operator +( BB3D left, BB3D right );

// adds positions and angles, uses left operands size
BB3D& operator +=( BB3D& left, BB3D right );
