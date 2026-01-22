#include <iostream>
#include <cmath>
#include <vector>
#include <fstream>
#include <algorithm>
#include <chrono> // throttling of packet sends
#include "aircraft.hpp"
#include "flight_visualizer.h"
#include "autopilot.cpp"
#include "comms_module.hpp"

// Constants
const double g = 9.81;  // gravity (m/s^2)
const double rho = 1.225;  // air density at sea level (kg/m^3)
bool flag_reset_sim = false;


// Helper function to create a simple 2D table
AeroTable2D createTable(const std::vector<double>& alphas, 
                        const std::vector<double>& betas,
                        const std::vector<std::vector<double>>& data)
{
    AeroTable2D table;
    table.alphas = alphas;
    table.betas = betas;
    table.data = data;
    return table;
}

// populates the aero model of the input aircraft
void MakeCessna( Aircraft& cessna )
{
    cessna.mass = 1000;      // kg
    cessna.S = 16.2;         // m^2
    cessna.chord = 1.5;      // m
    cessna.span = 11.0;      // m
    cessna.Ixx = 1285;       // kg*m^2
    cessna.Iyy = 1824;       // kg*m^2
    cessna.Izz = 2666;       // kg*m^2
    cessna.Ixz = 0;          // kg*m^2
    
    // Define angle ranges
    std::vector<double> alphas = {-0.174, 0.0, 0.087, 0.174, 0.262, 0.349};  // -10 to 20 deg
    std::vector<double> betas = {-0.174, -0.087, 0.0, 0.087, 0.174};  // -10 to 10 deg
    
    // CL table (varies strongly with alpha, weakly with beta)
    cessna.CL_table = createTable(alphas, betas, {
        {-0.32, -0.30, -0.30, -0.30, -0.32},  // alpha = -10°
        { 0.18,  0.20,  0.20,  0.20,  0.18},  // alpha =   0°
        { 0.68,  0.70,  0.70,  0.70,  0.68},  // alpha =   5°
        { 1.08,  1.10,  1.10,  1.10,  1.08},  // alpha =  10°
        { 1.28,  1.30,  1.30,  1.30,  1.28},  // alpha =  15°
        { 1.18,  1.20,  1.20,  1.20,  1.18}   // alpha =  20° (post-stall)
    });
    
    // CD table (increases with alpha and |beta|)
    cessna.CD_table = createTable(alphas, betas, {
        {0.035, 0.032, 0.030, 0.032, 0.035},  // alpha = -10°
        {0.028, 0.026, 0.025, 0.026, 0.028},  // alpha =   0°
        {0.032, 0.031, 0.030, 0.031, 0.032},  // alpha =   5°
        {0.044, 0.042, 0.040, 0.042, 0.044},  // alpha =  10°
        {0.085, 0.082, 0.080, 0.082, 0.085},  // alpha =  15°
        {0.155, 0.152, 0.150, 0.152, 0.155}   // alpha =  20°
    });
    
    // CY table (side force, mainly from beta)
    cessna.CY_table = createTable(alphas, betas, {
        {0.30, 0.15, 0.0, -0.15, -0.30},
        {0.30, 0.15, 0.0, -0.15, -0.30},
        {0.30, 0.15, 0.0, -0.15, -0.30},
        {0.30, 0.15, 0.0, -0.15, -0.30},
        {0.30, 0.15, 0.0, -0.15, -0.30},
        {0.30, 0.15, 0.0, -0.15, -0.30}
    });
    
    // Cl table (rolling moment, mainly from beta due to dihedral)
    cessna.Cl_table = createTable(alphas, betas, {
        {0.015, 0.008, 0.0, -0.008, -0.015},
        {0.015, 0.008, 0.0, -0.008, -0.015},
        {0.015, 0.008, 0.0, -0.008, -0.015},
        {0.015, 0.008, 0.0, -0.008, -0.015},
        {0.015, 0.008, 0.0, -0.008, -0.015},
        {0.015, 0.008, 0.0, -0.008, -0.015}
    });
    
    // Cm table (pitching moment)
    cessna.Cm_table = createTable(alphas, betas, {
        { 0.00,  0.00,  0.00,  0.00,  0.00},
        { 0.00,  0.00,  0.00,  0.00,  0.00},
        { 0.018, 0.020, 0.020, 0.020, 0.018},
        { 0.038, 0.040, 0.040, 0.040, 0.038},
        { 0.048, 0.050, 0.050, 0.050, 0.048},
        { 0.058, 0.060, 0.060, 0.060, 0.058}
    });
    
    // Cn table (yawing moment, weathercock stability)
    cessna.Cn_table = createTable(alphas, betas, {
        {-0.020, -0.010, 0.0, 0.010, 0.020},
        {-0.020, -0.010, 0.0, 0.010, 0.020},
        {-0.020, -0.010, 0.0, 0.010, 0.020},
        {-0.020, -0.010, 0.0, 0.010, 0.020},
        {-0.020, -0.010, 0.0, 0.010, 0.020},
        {-0.020, -0.010, 0.0, 0.010, 0.020}
    });
    
    // Control derivatives
    cessna.controls.CL_de = 0.4;   // Elevator lift effectiveness
    cessna.controls.Cm_de = -1.2;  // Elevator pitch effectiveness (negative = pitch down with up elevator)
    cessna.controls.Cl_da = 0.15;  // Aileron roll effectiveness
    cessna.controls.Cn_da = -0.01; // Adverse yaw
    cessna.controls.CY_dr = 0.3;   // Rudder side force
    cessna.controls.Cl_dr = 0.01;  // Rudder roll coupling
    cessna.controls.Cn_dr = -0.1;  // Rudder yaw effectiveness
}


// Calculate derivatives
void derivatives(const State& s, const Aircraft& ac, const ControlSignal& cs, State& ds)
{
    // Airspeed and flow angles
    double V = sqrt(s.u*s.u + s.v*s.v + s.w*s.w);
    if (V < 1e-3) V = 1e-3;  // Prevent division by zero
    
    double alpha = atan2(s.w, s.u);
    double beta = asin(std::max(-1.0, std::min(1.0, s.v / V)));
    
    // Dynamic pressure
    double qbar = 0.5 * rho * V * V;
    
    // Look up base aerodynamic coefficients
    double CL_base = ac.CL_table.lookup(alpha, beta);
    double CD_base = ac.CD_table.lookup(alpha, beta);
    double CY_base = ac.CY_table.lookup(alpha, beta);
    double Cl_base = ac.Cl_table.lookup(alpha, beta);
    double Cm_base = ac.Cm_table.lookup(alpha, beta);
    double Cn_base = ac.Cn_table.lookup(alpha, beta);
    
    // Add control surface effects
    double CL = CL_base + ac.controls.CL_de * cs.elevator;
    double CD = CD_base;  // Drag increases slightly with controls (ignored for simplicity)
    double CY = CY_base + ac.controls.CY_dr * cs.rudder;
    double Cl = Cl_base + ac.controls.Cl_da * cs.aileron + ac.controls.Cl_dr * cs.rudder;
    double Cm = Cm_base + ac.controls.Cm_de * cs.elevator;
    double Cn = Cn_base + ac.controls.Cn_da * cs.aileron + ac.controls.Cn_dr * cs.rudder;
    
    // Forces in stability axes (wind frame)
    double L = qbar * ac.S * CL;  // Lift (perpendicular to V)
    double D = qbar * ac.S * CD;  // Drag (parallel to V)
    double Y = qbar * ac.S * CY;  // Side force
    
    // Transform to body frame
    // Stability to body: rotate by alpha about y-axis, then beta about z-axis
    double ca = cos(alpha);
    double sa = sin(alpha);
    double cb = cos(beta);
    double sb = sin(beta);
    
    // Forces in body frame (simplified rotation)
    double Fx_aero = -D * ca * cb - Y * sb + L * sa * cb;
    double Fy_aero = -D * ca * sb + Y * cb + L * sa * sb;
    double Fz_aero = -D * sa - L * ca;
    
    // Thrust (simple model - along x-axis)
    double thrust = cs.throttle * ac.mass * g * 0.35;  // 35% thrust-to-weight
    double Fx_thrust = thrust;
    
    // Total aerodynamic + thrust forces
    double Fx = Fx_aero + Fx_thrust;
    double Fy = Fy_aero;
    double Fz = Fz_aero;
    
    // Gravity in body frame
    double sp = sin(s.phi);
    double cp = cos(s.phi);
    double st = sin(s.theta);
    double ct = cos(s.theta);
    
    Fx += -ac.mass * g * st;
    Fy += ac.mass * g * ct * sp;
    Fz += ac.mass * g * ct * cp;
    
    // Linear accelerations (body frame equations of motion)
    ds.u = Fx / ac.mass + s.r * s.v - s.q * s.w;
    ds.v = Fy / ac.mass - s.r * s.u + s.p * s.w;
    ds.w = Fz / ac.mass + s.q * s.u - s.p * s.v;
    
    // Moments in body frame
    double Ell = qbar * ac.S * ac.span * Cl;  // Rolling moment
    double M = qbar * ac.S * ac.chord * Cm;   // Pitching moment
    double N = qbar * ac.S * ac.span * Cn;    // Yawing moment
    
    // Angular accelerations (Euler's equations for rigid body)
    double denom = ac.Ixx * ac.Izz - ac.Ixz * ac.Ixz;
    
    double c1 = ((ac.Iyy - ac.Izz) * ac.Izz - ac.Ixz * ac.Ixz) / denom;
    double c2 = (ac.Ixz * (ac.Ixx - ac.Iyy + ac.Izz)) / denom;
    double c3 = ac.Izz / denom;
    double c4 = ac.Ixz / denom;
    double c5 = (ac.Izz - ac.Ixx) / ac.Iyy;
    double c6 = ac.Ixz / ac.Iyy;
    double c7 = ((ac.Ixx - ac.Iyy) * ac.Ixx + ac.Ixz * ac.Ixz) / denom;
    double c8 = ac.Ixx / denom;
    
    ds.p = c1 * s.q * s.r + c2 * s.p * s.q + c3 * Ell + c4 * N;
    ds.q = c5 * s.p * s.r - c6 * (s.p * s.p - s.r * s.r) + M / ac.Iyy;
    ds.r = c7 * s.p * s.q - c2 * s.q * s.r + c4 * Ell + c8 * N;
    
    // Euler angle rates
    double tt = tan(s.theta);
    ds.phi = s.p + sp * tt * s.q + cp * tt * s.r;
    ds.theta = cp * s.q - sp * s.r;
    ds.psi = (sp / ct) * s.q + (cp / ct) * s.r;
    
    // Position derivatives (body to NED frame)
    double cth = ct;
    double sth = st;
    double cps = cos(s.psi);
    double sps = sin(s.psi);
    
    ds.x = cth * cps * s.u + (sp * sth * cps - cp * sps) * s.v +
           (cp * sth * cps + sp * sps) * s.w;
    ds.y = cth * sps * s.u + (sp * sth * sps + cp * cps) * s.v +
           (cp * sth * sps - sp * cps) * s.w;
    ds.z = -sth * s.u + sp * cth * s.v + cp * cth * s.w;
}

// RK4 integration step
void integrate(State& s, const Aircraft& ac, double dt, const ControlSignal& cs)
{
    State k1, k2, k3, k4, temp;
    
    derivatives(s, ac, cs, k1);
    
    #define ADD_STATE(dst, src, k, scale) \
        dst.x = src.x + scale * k.x; \
        dst.y = src.y + scale * k.y; \
        dst.z = src.z + scale * k.z; \
        dst.u = src.u + scale * k.u; \
        dst.v = src.v + scale * k.v; \
        dst.w = src.w + scale * k.w; \
        dst.phi = src.phi + scale * k.phi; \
        dst.theta = src.theta + scale * k.theta; \
        dst.psi = src.psi + scale * k.psi; \
        dst.p = src.p + scale * k.p; \
        dst.q = src.q + scale * k.q; \
        dst.r = src.r + scale * k.r;
    
    ADD_STATE(temp, s, k1, dt/2);
    derivatives(temp, ac, cs, k2);
    
    ADD_STATE(temp, s, k2, dt/2);
    derivatives(temp, ac, cs, k3);
    
    ADD_STATE(temp, s, k3, dt);
    derivatives(temp, ac, cs, k4);
    
    #define INTEGRATE(field) \
        s.field += (dt/6.0) * (k1.field + 2*k2.field + 2*k3.field + k4.field);
    
    INTEGRATE(x); INTEGRATE(y); INTEGRATE(z);
    INTEGRATE(u); INTEGRATE(v); INTEGRATE(w);
    INTEGRATE(phi); INTEGRATE(theta); INTEGRATE(psi);
    INTEGRATE(p); INTEGRATE(q); INTEGRATE(r);
}

// log to std::cout
void printLog( const State& state, double simTime, std::ofstream& outFile )
{
    // Output state
    double V = sqrt(state.u*state.u + state.v*state.v + state.w*state.w);
    double alpha = atan2(state.w, state.u) * 180.0 / M_PI;
    double beta = asin(std::max(-1.0, std::min(1.0, state.v / V))) * 180.0 / M_PI;
    
    outFile << simTime << "," << state.x << "," << state.y << "," 
           << -state.z << "," << state.u << "," << state.v << "," 
           << state.w << "," << state.phi << "," << state.theta << "," 
           << state.psi << "," << state.p << "," << state.q << "," 
           << state.r << "," << V << "," << alpha << "," << beta << "\n";
    
    std::cout << "t=" << simTime << "s | Alt=" << -state.z 
             << "m | V=" << V << "m/s | α=" << alpha 
             << "° | β=" << beta << "° | φ=" << state.phi * 180/M_PI << "°\n";
}

// sends simulation telemetry to command console
bool sendTelemetry( CommsModule& comms, const State& s )
{
    static CommsPacket tx_packet( CommsPacket::sim_telemetry );
    // phi -> -PI +PI
    float phi = fmod(s.phi+PI, 2*PI)-PI;
    // theta -> -PI/2 +PI/2
    float theta = fmod(s.theta+PI/2, PI)-(PI/2);
    // psi -> 0 +2PI
    float psi = fmod(s.psi, 2*PI);

    tx_packet.st_setAS_z    ( s.z );
    tx_packet.st_setAS_u    ( s.u );
    tx_packet.st_setAS_w    ( s.w );
    tx_packet.st_setAS_phi  ( phi );
    tx_packet.st_setAS_theta( theta );
    tx_packet.st_setAS_psi  ( psi );
    return comms.sendPacket( tx_packet, CommsModule::console_channel );
}

bool receiveCommands( CommsModule& comms, ControlSignal& cs )
{
    static CommsPacket rx_packet( CommsPacket::console_sim_command );
    if( !comms.readPacket( rx_packet ) ) return false;
    if( rx_packet.packet_type != CommsPacket::console_sim_command ) return false;

    /*
    // DEBUG
    static uint8_t raw[32];
    rx_packet.getRaw( raw );
    std::cout<<"received packet: ";
    for(int i=0; i<32; i++)
        std::cout<<(int)raw[i]<<" ";
    std::cout<<"\n";
    // DEBUG END
    */

    cs.throttle = rx_packet.csc_getCS_throttle();
    cs.elevator = rx_packet.csc_getCS_elevator();
    cs.aileron  = rx_packet.csc_getCS_aileron();
    cs.rudder   = rx_packet.csc_getCS_rudder();
    AP::mode = rx_packet.csc_getCS_APMode();
    flag_reset_sim = rx_packet.csc_getCS_resetSim();

    // tame it down a bit
    cs.elevator *= 0.3f;
    cs.aileron *= 0.3f;
    cs.rudder *= 0.3f;
    return true;
}

int main()
{
    // Define aircraft
    Aircraft cessna;
    MakeCessna( cessna );
    
    // Initial state - level flight
    State state = {0};
    state.u = 50.0;   // 50 m/s forward
    state.z = -100;  // 1000m altitude (NED: down is positive)
    state.theta = 0.05;  // Small pitch up for trim
    
    // Control inputs
    ControlSignal control_s;
    ControlSignal pilot_cs;

    // Simulation parameters
    double dt = 0.01;  // 10ms timestep
    double simTime = 0.0;

    // Comms with command console
    CommsModule comms( CommsModule::sim_channel );
    std::chrono::steady_clock clock;
    std::chrono::time_point<std::chrono::steady_clock> last_transmission_time;
    std::chrono::milliseconds transmission_interval(100);

    // GUI
    FlightVisualizer viz(1280, 720);
    viz.Initialize("Flight Sim GUI");
    
    // Open output file
    std::ofstream outFile("flight_data.csv");
    outFile << "time,x,y,z,u,v,w,phi,theta,psi,p,q,r,airspeed,alpha,beta\n";
    
    std::cout << "Starting full 6-DOF flight simulation...\n";
    std::cout << "Aerodynamic model: CL, CD, CY, Cl, Cm, Cn = f(alpha, beta)\n\n";
    
    while (true)    // inf loop of simulation
    {
        if( viz.ShouldClose() ) break;  // GUI wanted to end program

        /*
        // output state to cout
        if (fmod(simTime, 0.1) < dt)
            printLog( state, simTime, outFile );
        */
        
        // receive commands thru comms module
        while( comms.packetAvailable() ) receiveCommands( comms, pilot_cs );

        // we hold the last pilot cs steady
        control_s = pilot_cs;

        // autopilot does magic here
        AP::run( state, control_s );

        // DEBUG
        if (IsKeyPressed(KEY_R) || flag_reset_sim )
        { // reset flight state
            state = {0};
            state.u = 80.0;   // 50 m/s forward
            state.z = -100;  // 1000m altitude (NED: down is positive)
            state.theta = 0.05;  // Small pitch up for trim
            flag_reset_sim = false;
        }
        
        // Integrate
        integrate(state, cessna, dt, control_s );
        
        simTime += dt;
        
        // Ground check
        if (state.z > 0)
        {
            std::cout << "\nAircraft hit ground!\n";
            break;
        }

        // send telemetry back to command console
        if (clock.now() >= last_transmission_time + transmission_interval) {
            sendTelemetry( comms, state );
            last_transmission_time = clock.now();
        }

        // GUI
        viz.Update(state);
        viz.Draw(state, control_s, AP::mode);

    }
    
    // GUI
    viz.Close();
    
    outFile.close();
    std::cout << "\nSimulation complete. Data saved to flight_data.csv\n";
    std::cout << "Plot alpha, beta, phi, theta, psi to see flight dynamics!\n";
    
    return 0;
}
