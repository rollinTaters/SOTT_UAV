#pragma once
//#include <cstdint>    avr does not like this c++ library, we are instead using the c version stdint.h
#include <stdint.h>
//#include <cstring>  // memcpy, avr likes c libs, so using string.h instead
#include <string.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

struct CommsPacket
{
    enum PacketType{
        undefined,

        turret,             // MC -> FC(servo driver)
        flight_control,     // console -> FC
        fc_telemetry,       // FC -> console/MC

        ngc_command,        // MC -> NGC
        ngc_telemetry,      // NGC -> MC

        video_packet,       // MC -> FC -> console
        console_telemetry,  // console -> FC/MC
        console_command,    // console -> MC
        console_cammand_custom,  // console -> MC

        request1,   // this is an idea
        request2,   // this is an idea
        request3    // this is an idea
    };


    // actual payload of the packet
    uint8_t packet_type = undefined;
    uint8_t data[31] = {0};

    CommsPacket( PacketType t ):packet_type(t) {}
    CommsPacket():packet_type(undefined) {}


    private:
    void writeFloat_1( float input, int offset, float minval, float maxval )
    {
        // clamp it
        input = (( input>minval )?(input):(minval));
        input = (( input<maxval )?(input):(maxval));

        // make the input fit between designated sizes value range
        float range = maxval - minval;
        uint8_t byte1 = (uint8_t)( (input-minval)/range *255 );

        memcpy( &(data[offset]), &byte1, sizeof( byte1 ) );
    }

    float readFloat_1( int offset, float minval, float maxval ) const
    {
        uint8_t byte1;
        memcpy( &byte1, &(data[offset]), sizeof( byte1 ) );

        float range = maxval - minval;
        return ( byte1/255.f * range ) + minval;
    }

    void writeFloat_2( float input, int offset, float minval, float maxval )
    {
        // clamp it
        input = (( input>minval )?(input):(minval));
        input = (( input<maxval )?(input):(maxval));

        // make the input fit between designated sizes value range
        float range = maxval - minval;
        uint16_t byte2 = (uint16_t)( (input-minval)/range *65535 );

        memcpy( &(data[offset]), &byte2, sizeof( byte2 ) );
    }

    float readFloat_2( int offset, float minval, float maxval ) const
    {
        uint16_t byte2;
        memcpy( &byte2, &(data[offset]), sizeof( byte2 ) );

        float range = maxval - minval;
        return ( byte2/65535.f * range ) + minval;
    }

    void write_2( uint16_t input, int offset )
    {
        uint8_t b1 = (uint8_t)(0xFF & input);
        uint8_t b2 = (uint8_t)(input >> 8);
        data[offset] = b1;
        data[offset+1] = b2;
    }

    uint16_t read_2( int offset ) const
    {
        uint16_t b1 = data[offset];
        uint16_t b2 = data[offset+1];
        return ( b1 | (b2 << 8) );
    }

    void write_1( uint8_t input, int offset )
    {
        data[offset] = input;
    }

    uint8_t read_1( int offset ) const
    {
        return data[offset];
    }
    public:

    bool isNull() const
    {
        // if there is data, its not null packet
        for( uint8_t d : data )
            if( d != 0 )
                return false;

        // if there is no data and packet type is undefined
        if( packet_type == undefined ) return true;

        // if there is no data, but packet type is defined,
        return false;
    }

/*
   These are different methods that we use to parse (encode/decode) the data in the packets.
   we use these packets to transfer data between different subsystems
   Feel free to add your own packet parser methods if needed
*/

    /* XXX there is no fucking drive module. electronics team is indecisive 
    // ---- data sent to the drive module ----
    float getSpeed(){ return data1; }
    float getSteer(){ return data2; }

    void setSpeed( float inp ){ data1 = inp; }
    void setSteer( float inp ){ data2 = inp; }

    // ---- data sent from drive module ----
    float getMotor1Temp(){ return data1; }
    float getMotor1Amps(){ return data2; }
    float getMotor1Vel() { return data3; }
    float getMotor2Temp(){ return data1; }
    float getMotor2Amps(){ return data2; }
    float getMotor2Vel() { return data3; }

    void setMotor1Temp( float inp ){ data1 = inp; }
    void setMotor1Amps( float inp ){ data2 = inp; }
    void setMotor1Vel ( float inp ){ data3 = inp; }
    void setMotor2Temp( float inp ){ data1 = inp; }
    void setMotor2Amps( float inp ){ data2 = inp; }
    void setMotor2Vel ( float inp ){ data3 = inp; }
    */



    /* TODO
    // ---- data sent/received to/from the turret module ----
    float getPan() { return data1; }
    float getTilt(){ return data2; }
    bool getLaserStatus(){ return data3; }

    void setPan ( float inp ){ data1 = inp; }
    void setTilt( float inp ){ data2 = inp; }
    void setLaserStatus( bool inp ){ data3 = inp; }
    */

    /* TODO
    // ---- data sent to the ngc module ----
    uint8_t getControlMode(){ return data1; }

    void setControlMode( uint8_t inp ){ data1 = inp; }
    */

    /* TODO
    // ---- NGC TELEMETRY: from NGC to CCM ----
    float getHeading(){ return data1; }
    float getPitch()  { return data2; }
    float getRoll()   { return data3; }

    void setHeading( float inp ){ data1 = inp; }
    void setPitch  ( float inp ){ data2 = inp; }
    void setRoll   ( float inp ){ data3 = inp; }
    */

    // ----  video data packet ----
    uint16_t getFrameID() const { return read_2( 0 ); }
    uint16_t getChunkID() const { return read_2( 2 ); }
    uint16_t getFrameSize() const { return read_2( 4 ); }  // this has one more byte to expand to
    //uint16_t getTotalChunks() const { return data3;} // optional
    void getPayload( uint8_t* le_array ) const
    {
        // copies 25 bytes of data to input array
        memcpy( le_array, &(data[6]), sizeof(uint8_t)*25 );
    }

    void setFrameID(uint16_t id) { write_2( id, 0 ); }
    void setChunkID(uint16_t id) { write_2( id, 2 ); }
    void setFrameSize(uint16_t size) { write_2( size, 4 ); }
    //void setTotalChunks(uint8_t chunks) { data3 = chunks; } // optional
    void setPayload(const uint8_t* inp_data)
    {
        memcpy( &(data[6]), inp_data, sizeof(uint8_t)*25 );
    }

    // ---- Console Telemetry, from CCM to Console ----
    float ct_getMotor1Temp(){ return readFloat_1( 0, -20.f, 250.f ); }  // celsius
    float ct_getMotor2Temp(){ return readFloat_1( 1, -20.f, 250.f ); }  // celsius
    float ct_getMotor1Amps(){ return readFloat_1( 2, -10.f, 300.f ); }  // ampere
    float ct_getMotor2Amps(){ return readFloat_1( 3, -10.f, 300.f ); }  // ampere 
    float ct_getMotor1Vel() { return readFloat_1( 4, -5.f, 5.f ); }     // m/s
    float ct_getMotor2Vel() { return readFloat_1( 5, -5.f, 5.f ); }     // m/s
    float ct_getHeading(){ return readFloat_2(  6, 0.f, PI ); }      // radian
    float ct_getPitch()  { return readFloat_2(  8, -PI/2, PI/2 ); }  // radian
    float ct_getRoll()   { return readFloat_2( 10, -PI/2, PI/2 ); }  // radian
    float ct_getSpeed()  { return readFloat_2( 12, -5.f, 5.f ); }    // m/s


    void ct_setMotor1Temp( float inp ){ writeFloat_1( inp, 0, -20.f, 250.f ); } // celsius
    void ct_setMotor2Temp( float inp ){ writeFloat_1( inp, 1, -20.f, 250.f ); } // celsius
    void ct_setMotor1Amps( float inp ){ writeFloat_1( inp, 2, -10.f, 300.f ); } // ampere
    void ct_setMotor2Amps( float inp ){ writeFloat_1( inp, 3, -10.f, 300.f ); } // ampere
    void ct_setMotor1Vel ( float inp ){ writeFloat_1( inp, 4, -5.f, 5.f ); }    // m/s
    void ct_setMotor2Vel ( float inp ){ writeFloat_1( inp, 5, -5.f, 5.f ); }    // m/s
    void ct_setHeading( float inp ){ writeFloat_2( inp,  6, 0.f, PI ); }      // radian
    void ct_setPitch  ( float inp ){ writeFloat_2( inp,  8, -PI/2, PI/2 ); }  // radian
    void ct_setRoll   ( float inp ){ writeFloat_2( inp, 10, -PI/2, PI/2 ); }  // radian
    void ct_setSpeed  ( float inp ){ writeFloat_2( inp, 12, -5.f, 5.f ); }    // m/s   

    // ---- Console Command, from Console to CCM ----
    void cc_setManualSpeed( float inp ){ writeFloat_2( inp, 0, -5.f, 5.f ); }  // m/s
    void cc_setManualSteer( float inp ){ writeFloat_2( inp, 2, -1.f, 1.f );}   // unitless

    float cc_getManualSpeed(){ return readFloat_2( 0, -5.f, 5.f ); }   // m/s
    float cc_getManualSteer(){ return readFloat_2( 2, -1.f, 1.f ); }   // unitless

    void cc_setTurret_azimuth( float rad ) { writeFloat_2( rad, 4, 0, 2*PI ); }
    void cc_setTurret_elevation( float rad ) { writeFloat_2( rad, 6, -PI, PI ); }
    float cc_getTurret_azimuth() { return readFloat_2( 4, 0, 2*PI ); }
    float cc_getTurret_elevation() { return readFloat_2( 6, -PI, PI ); }
    
    void cc_setLaserStatus( uint8_t mode ) { write_1( mode, 8 ); }

    // alternate packet type, this is CommsPacket::console_command_custom
    // Waypoint commands
    //void ccc_setAddWaypoint( float x, float y, float z ) { /* TODO */ }
    //void ccc_getAddWaypoint( float &x, float &y, float &z ) const { /* TODO */ }

};

