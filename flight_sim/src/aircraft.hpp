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
