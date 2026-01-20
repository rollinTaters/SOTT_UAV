#pragma once
#include <vector>

// 2D lookup table for aerodynamic coefficients
struct AeroTable2D
{
    std::vector<double> alphas;  // angle of attack values (radians)
    std::vector<double> betas;   // sideslip angle values (radians)
    std::vector<std::vector<double>> data;  // data[alpha_idx][beta_idx]
    
    // Bilinear interpolation
    double lookup(double alpha, double beta) const
    {
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

struct ControlSignal
{
    double throttle = 0.4; // was 0.5 initially
    double elevator = 0.0;
    double aileron = 0.0;
    double rudder = 0.0;
};

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

