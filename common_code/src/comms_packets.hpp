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

        sim_telemetry,      // sim -> console
        console_sim_command,// console -> sim

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
    CommsPacket( uint8_t raw[32] ):packet_type(raw[0])
    { memcpy( &(data[0]), &(raw[1]), 31 ); }


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

    void getRaw( uint8_t* raw ) const
    {
        raw[0] = packet_type;
        memcpy( &(raw[1]), &(data[0]), 31 );
    }

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
    // ---- Console Command ----
    // console -> sim
    // joystick: ControlSignal: throttle, elevator, aileron, rudder
    // joystick: reset_sim, set AP mode
    float   csc_getCS_throttle() const { return readFloat_2( 0, -1.f, 1.f ); }
    float   csc_getCS_elevator() const { return readFloat_2( 2, -1.f, 1.f ); }
    float   csc_getCS_aileron () const { return readFloat_2( 4, -1.f, 1.f ); }
    float   csc_getCS_rudder  () const { return readFloat_2( 6, -1.f, 1.f ); }
    uint8_t csc_getCS_resetSim() const { return read_1(8);}
    uint8_t csc_getCS_APMode  () const { return read_1(9); }

    void csc_setCS_throttle( float   inp ){ writeFloat_2( inp, 0, -1.f, 1.f ); }
    void csc_setCS_elevator( float   inp ){ writeFloat_2( inp, 2, -1.f, 1.f ); }
    void csc_setCS_aileron ( float   inp ){ writeFloat_2( inp, 4, -1.f, 1.f ); }
    void csc_setCS_rudder  ( float   inp ){ writeFloat_2( inp, 6, -1.f, 1.f ); }
    void csc_setCS_resetSim( bool    inp ){ write_1( inp, 8 ); }
    void csc_setCS_APMode  ( uint8_t inp ){ write_1( inp, 9 ); }



    // ---- Simulation Telemetry ----
    // sim -> console
    // aircraft state: z, u, w, phi, theta, psi,
    float st_getAS_z    () const { return readFloat_2( 0, -10000.f, 0.f ); }
    float st_getAS_u    () const { return readFloat_2( 2, -10.f, 100.f ); }
    float st_getAS_w    () const { return readFloat_2( 4, -100.f, 100.f ); }
    float st_getAS_phi  () const { return readFloat_2( 6, -PI, PI ); }
    float st_getAS_theta() const { return readFloat_2( 8, -PI/2, PI/2 ); }
    float st_getAS_psi  () const { return readFloat_2(10, 0, 2*PI ); }

    void st_setAS_z    ( float inp ){ writeFloat_2( inp, 0, -10000.f, 0.f ); }
    void st_setAS_u    ( float inp ){ writeFloat_2( inp, 2, -10.f, 100.f ); }
    void st_setAS_w    ( float inp ){ writeFloat_2( inp, 4, -100.f, 100.f ); }
    void st_setAS_phi  ( float inp ){ writeFloat_2( inp, 6, -PI, PI ); }
    void st_setAS_theta( float inp ){ writeFloat_2( inp, 8, -PI/2, PI/2 ); }
    void st_setAS_psi  ( float inp ){ writeFloat_2( inp,10, 0, 2*PI ); }




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
    float ct_getMotor1Temp() const { return readFloat_1( 0, -20.f, 250.f ); }  // celsius
    float ct_getMotor2Temp() const { return readFloat_1( 1, -20.f, 250.f ); }  // celsius
    float ct_getMotor1Amps() const { return readFloat_1( 2, -10.f, 300.f ); }  // ampere
    float ct_getMotor2Amps() const { return readFloat_1( 3, -10.f, 300.f ); }  // ampere 
    float ct_getMotor1Vel()  const { return readFloat_1( 4, -5.f, 5.f ); }     // m/s
    float ct_getMotor2Vel()  const { return readFloat_1( 5, -5.f, 5.f ); }     // m/s
    float ct_getHeading()    const { return readFloat_2(  6, 0.f, PI ); }      // radian
    float ct_getPitch()      const { return readFloat_2(  8, -PI/2, PI/2 ); }  // radian
    float ct_getRoll()       const { return readFloat_2( 10, -PI/2, PI/2 ); }  // radian
    float ct_getSpeed()      const { return readFloat_2( 12, -5.f, 5.f ); }    // m/s


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

    /* OBSOLETE // ---- Console Command, from Console to CCM ----
    void cc_setTurret_azimuth( float rad ) { writeFloat_2( rad, 4, 0, 2*PI ); }
    void cc_setTurret_elevation( float rad ) { writeFloat_2( rad, 6, -PI, PI ); }
    float cc_getTurret_azimuth() { return readFloat_2( 4, 0, 2*PI ); }
    float cc_getTurret_elevation() { return readFloat_2( 6, -PI, PI ); }
    
    void cc_setLaserStatus( uint8_t mode ) { write_1( mode, 8 ); }
    */

    // alternate packet type, this is CommsPacket::console_command_custom
    // Waypoint commands
    //void ccc_setAddWaypoint( float x, float y, float z ) { /* TODO */ }
    //void ccc_getAddWaypoint( float &x, float &y, float &z ) const { /* TODO */ }

};

