#include <iostream>
#include <cmath>
#include <vector>
#include <fstream>
#include <algorithm>
#include "flight_visualizer.h"

// Constants
const double g = 9.81;  // gravity (m/s^2)
const double rho = 1.225;  // air density at sea level (kg/m^3)

// 2D lookup table for aerodynamic coefficients
struct AeroTable2D
{
    std::vector<double> alphas;  // angle of attack values (radians)
    std::vector<double> betas;   // sideslip angle values (radians)
    std::vector<std::vector<double>> data;  // data[alpha_idx][beta_idx]
    
    // Bilinear interpolation
    double lookup(double alpha, double beta) const {
        // Clamp to table bounds
        alpha = std::max(alphas.front(), std::min(alphas.back(), alpha));
        beta = std::max(betas.front(), std::min(betas.back(), beta));
        
        // Find bracketing indices for alpha
        size_t i0 = 0;
        for (size_t i = 0; i < alphas.size() - 1; i++) {
            if (alpha >= alphas[i] && alpha <= alphas[i+1]) {
                i0 = i;
                break;
            }
        }
        
        // Find bracketing indices for beta
        size_t j0 = 0;
        for (size_t j = 0; j < betas.size() - 1; j++) {
            if (beta >= betas[j] && beta <= betas[j+1]) {
                j0 = j;
                break;
            }
        }
        
        // Bilinear interpolation
        double t_alpha = (alpha - alphas[i0]) / (alphas[i0+1] - alphas[i0]);
        double t_beta = (beta - betas[j0]) / (betas[j0+1] - betas[j0]);
        
        double f00 = data[i0][j0];
        double f10 = data[i0+1][j0];
        double f01 = data[i0][j0+1];
        double f11 = data[i0+1][j0+1];
        
        double f0 = f00 + t_alpha * (f10 - f00);
        double f1 = f01 + t_alpha * (f11 - f01);
        
        return f0 + t_beta * (f1 - f0);
    }
};

// Control derivatives (linear approximation)
struct ControlDerivatives
{
    // Elevator effects
    double CL_de;   // dCL/d(elevator)
    double Cm_de;   // dCm/d(elevator)
    
    // Aileron effects
    double Cl_da;   // dCl/d(aileron)
    double Cn_da;   // dCn/d(aileron) - adverse yaw
    
    // Rudder effects
    double CY_dr;   // dCY/d(rudder)
    double Cl_dr;   // dCl/d(rudder)
    double Cn_dr;   // dCn/d(rudder)
};

/*
// Aircraft state vector
struct State
{
    // Position (NED frame)
    double x, y, z;  // position (m)
    
    // Velocity (body frame)
    double u, v, w;  // body frame velocities (m/s)
    
    // Orientation (Euler angles)
    double phi, theta, psi;  // roll, pitch, yaw (radians)
    
    // Angular rates (body frame)
    double p, q, r;  // roll, pitch, yaw rates (rad/s)
};
*/

// Aircraft parameters
struct Aircraft
{
    double mass;        // kg
    double S;          // wing area (m^2)
    double chord;      // mean aerodynamic chord (m)
    double span;       // wingspan (m)
    
    // Moments of inertia (kg*m^2)
    double Ixx, Iyy, Izz, Ixz;
    
    // Aerodynamic coefficient tables
    AeroTable2D CL_table;  // Lift coefficient
    AeroTable2D CD_table;  // Drag coefficient
    AeroTable2D CY_table;  // Side force coefficient
    AeroTable2D Cl_table;  // Rolling moment coefficient
    AeroTable2D Cm_table;  // Pitching moment coefficient
    AeroTable2D Cn_table;  // Yawing moment coefficient
    
    // Control derivatives
    ControlDerivatives controls;
};

// Calculate derivatives
void derivatives(const State& s, const Aircraft& ac, 
                double throttle, double elevator, double aileron, double rudder,
                State& ds)
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
    double CL = CL_base + ac.controls.CL_de * elevator;
    double CD = CD_base;  // Drag increases slightly with controls (ignored for simplicity)
    double CY = CY_base + ac.controls.CY_dr * rudder;
    double Cl = Cl_base + ac.controls.Cl_da * aileron + ac.controls.Cl_dr * rudder;
    double Cm = Cm_base + ac.controls.Cm_de * elevator;
    double Cn = Cn_base + ac.controls.Cn_da * aileron + ac.controls.Cn_dr * rudder;
    
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
    double thrust = throttle * ac.mass * g * 0.35;  // 35% thrust-to-weight
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
void integrate(State& s, const Aircraft& ac, double dt,
               double throttle, double elevator, double aileron, double rudder)
{
    State k1, k2, k3, k4, temp;
    
    derivatives(s, ac, throttle, elevator, aileron, rudder, k1);
    
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
    derivatives(temp, ac, throttle, elevator, aileron, rudder, k2);
    
    ADD_STATE(temp, s, k2, dt/2);
    derivatives(temp, ac, throttle, elevator, aileron, rudder, k3);
    
    ADD_STATE(temp, s, k3, dt);
    derivatives(temp, ac, throttle, elevator, aileron, rudder, k4);
    
    #define INTEGRATE(field) \
        s.field += (dt/6.0) * (k1.field + 2*k2.field + 2*k3.field + k4.field);
    
    INTEGRATE(x); INTEGRATE(y); INTEGRATE(z);
    INTEGRATE(u); INTEGRATE(v); INTEGRATE(w);
    INTEGRATE(phi); INTEGRATE(theta); INTEGRATE(psi);
    INTEGRATE(p); INTEGRATE(q); INTEGRATE(r);
}

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

int main()
{
    // Define aircraft
    Aircraft cessna;
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
    
    // Initial state - level flight
    State state = {0};
    state.u = 50.0;   // 50 m/s forward
    state.z = -1000;  // 1000m altitude (NED: down is positive)
    state.theta = 0.05;  // Small pitch up for trim
    
    // Simulation parameters
    double dt = 0.01;  // 10ms timestep
    double simTime = 0.0;
    double maxTime = 30.0;
    
    // Control inputs
    double throttle = 0.5;
    double elevator = 0.0;
    double aileron = 0.0;
    double rudder = 0.0;

    // GUI
    FlightVisualizer viz(1280, 720);
    viz.Initialize("Flight Sim GUI");
    
    // Open output file
    std::ofstream outFile("flight_data.csv");
    outFile << "time,x,y,z,u,v,w,phi,theta,psi,p,q,r,airspeed,alpha,beta\n";
    
    std::cout << "Starting full 6-DOF flight simulation...\n";
    std::cout << "Aerodynamic model: CL, CD, CY, Cl, Cm, Cn = f(alpha, beta)\n\n";
    
    while (!viz.ShouldClose())
    {
        if (simTime < maxTime)
        {
            // Output state
            if (fmod(simTime, 0.1) < dt)
            {
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
            
            // Simple autopilot example: maintain altitude and wings level
            elevator = -0.5 * (state.theta - 0.05) - 0.2 * state.q;  // Pitch hold
            aileron = -0.3 * state.phi - 0.1 * state.p;  // Roll hold
            rudder = -0.2 * state.r;  // Yaw damper
            
            // Example: coordinated turn at t=10s
            if (simTime > 10.0 && simTime < 15.0)
            {
                aileron = 0.2;  // Roll right
                rudder = 0.05;  // Coordinated rudder
            }
            
            // Integrate
            integrate(state, cessna, dt, throttle, elevator, aileron, rudder);
            
            simTime += dt;
            
            // Ground check
            if (state.z > 0)
            {
                std::cout << "\nAircraft hit ground!\n";
                break;
            }
        }

        // GUI
        viz.Update(state);
        viz.Draw(state);

    }
    
    // GUI
    viz.Close();
    
    outFile.close();
    std::cout << "\nSimulation complete. Data saved to flight_data.csv\n";
    std::cout << "Plot alpha, beta, phi, theta, psi to see flight dynamics!\n";
    
    return 0;
}
