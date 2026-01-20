#ifndef FLIGHT_VISUALIZER_H
#define FLIGHT_VISUALIZER_H

#include "raylib.h"
#include "raymath.h"
#include "aircraft.hpp"
#include <vector>
#include <deque>
#include <string>
#include <sstream>
#include <iomanip>

/*
// a quick reminder of what state looks like
struct State
{
    double x, y, z;
    double u, v, w;
    double phi, theta, psi;
    double p, q, r;
};
*/

class FlightVisualizer
{
private:
    int screenWidth;
    int screenHeight;
    Camera3D camera;
    
    // Trail history
    std::deque<Vector3> trailPoints;
    int maxTrailPoints;
    
    // UI state
    bool showGrid;
    bool showAxes;
    bool showTrail;
    bool showVelocity;
    bool showHUD;
    float cameraDistance;
    float cameraHeight;
    bool followAircraft;
    
    // Colors
    Color skyColor;
    Color groundColor;
    Color aircraftColor;
    Color trailColor;

    // Workaround - raylib only updates gamepad state before EndDrawing();
    ControlSignal pilot_control_signal;
    
public:
    FlightVisualizer(int width = 1280, int height = 720) 
        : screenWidth(width), screenHeight(height), maxTrailPoints(500),
          showGrid(true), showAxes(true), showTrail(true), 
          showVelocity(true), showHUD(true),
          cameraDistance(50.0f), cameraHeight(20.0f), followAircraft(true)
    {
        // Initialize colors
        skyColor = Color{135, 206, 235, 255};
        groundColor = Color{34, 139, 34, 255};
        aircraftColor = RED;
        trailColor = YELLOW;
    }
    
    void Initialize(const char* title = "Flight Simulator")
    {
        InitWindow(screenWidth, screenHeight, title);
        SetTargetFPS(60);
        
        // Setup camera
        camera.position = Vector3{50.0f, 30.0f, 50.0f};
        camera.target = Vector3{0.0f, 0.0f, 0.0f};
        camera.up = Vector3{0.0f, 1.0f, 0.0f};
        camera.fovy = 60.0f;
        camera.projection = CAMERA_PERSPECTIVE;
    }
    
    void Update(const State& state)
    {
        // Update camera to follow aircraft
        if (followAircraft) {
            Vector3 aircraftPos = {(float)state.x, (float)-state.z, (float)state.y};
            
            // Camera orbits behind and above the aircraft
            float psi = (float)state.psi;
            camera.position.x = aircraftPos.x - cameraDistance * cos(psi);
            camera.position.z = aircraftPos.z - cameraDistance * sin(psi);
            camera.position.y = aircraftPos.y + cameraHeight;
            camera.target = aircraftPos;
        } else {
        // Manual camera control when not following
            if (IsKeyDown(KEY_W)) camera.position.z -= 1.0f;
            if (IsKeyDown(KEY_S)) camera.position.z += 1.0f;
            if (IsKeyDown(KEY_A)) camera.position.x -= 1.0f;
            if (IsKeyDown(KEY_D)) camera.position.x += 1.0f;
            if (IsKeyDown(KEY_Q)) camera.position.y += 1.0f;
            if (IsKeyDown(KEY_E)) camera.position.y -= 1.0f;
        }
        
        // Add to trail
        Vector3 pos = {(float)state.x, (float)-state.z, (float)state.y};
        trailPoints.push_back(pos);
        if (trailPoints.size() > maxTrailPoints) {
            trailPoints.pop_front();
        }
        
        // Handle keyboard input for view controls
        if (IsKeyPressed(KEY_G)) showGrid = !showGrid;
        if (IsKeyPressed(KEY_A)) showAxes = !showAxes;
        if (IsKeyPressed(KEY_T)) showTrail = !showTrail;
        if (IsKeyPressed(KEY_V)) showVelocity = !showVelocity;
        if (IsKeyPressed(KEY_H)) showHUD = !showHUD;
        if (IsKeyPressed(KEY_F)) followAircraft = !followAircraft;
        
        // Camera distance control
        if (IsKeyDown(KEY_EQUAL)) cameraDistance -= 1.0f;
        if (IsKeyDown(KEY_MINUS)) cameraDistance += 1.0f;
        cameraDistance = Clamp(cameraDistance, 10.0f, 200.0f);
    }
    
    void Draw(const State& state, const ControlSignal& cs)
    {
        BeginDrawing();
        ClearBackground(skyColor);
        
        BeginMode3D(camera);
        
        // Draw ground plane
        DrawPlane(Vector3{0, 0, 0}, Vector2{1000, 1000}, groundColor);
        
        // Draw grid
        if (showGrid) {
            DrawGrid(100, 10.0f);
        }
        
        // Draw world axes
        if (showAxes) {
            DrawLine3D(Vector3{0, 0, 0}, Vector3{100, 0, 0}, RED);    // X - North
            DrawLine3D(Vector3{0, 0, 0}, Vector3{0, 100, 0}, BLUE);   // Y - Up
            DrawLine3D(Vector3{0, 0, 0}, Vector3{0, 0, 100}, GREEN);  // Z - East
        }
        
        // Draw aircraft
        DrawAircraft(state);
        
        // Draw velocity vector
        if (showVelocity) {
            DrawVelocityVector(state);
        }
        
        // Draw trail
        if (showTrail && trailPoints.size() > 1) {
            for (size_t i = 1; i < trailPoints.size(); i++) {
                float alpha = (float)i / trailPoints.size();
                Color c = ColorAlpha(trailColor, alpha);
                DrawLine3D(trailPoints[i-1], trailPoints[i], c);
            }
        }
        
        EndMode3D();
        
        // Draw HUD
        if (showHUD) {
            DrawHUD(state,cs);
        }
        
        // Draw controls help
        DrawControlsHelp();
        
        ReadJoystick( pilot_control_signal );
        EndDrawing();
    }

    void ReadJoystick( ControlSignal& cs )
    {
        int gamepad = 0;
        if( !IsGamepadAvailable(gamepad) )
        {
            std::cerr<<"No Gamepad connected!\n";
            return;
        }
        float elevator_dead_zone = 0.1f;
        float aileron_dead_zone  = 0.1f;
        float rudder_dead_zone   = 0.1f;
        cs.throttle = -GetGamepadAxisMovement( gamepad, GAMEPAD_AXIS_RIGHT_Y );
        cs.elevator = -GetGamepadAxisMovement( gamepad, GAMEPAD_AXIS_LEFT_Y  );
        cs.aileron  = GetGamepadAxisMovement( gamepad, GAMEPAD_AXIS_LEFT_X  );
        cs.rudder   = GetGamepadAxisMovement( gamepad, GAMEPAD_AXIS_RIGHT_X );
        if( cs.elevator > -elevator_dead_zone && cs.elevator < elevator_dead_zone ) cs.elevator = 0.0f;
        if( cs.aileron  > -aileron_dead_zone  && cs.aileron  < aileron_dead_zone  ) cs.aileron  = 0.0f;
        if( cs.rudder   > -rudder_dead_zone   && cs.rudder   < rudder_dead_zone   ) cs.rudder   = 0.0f;
    }

    void GetControlInputs( ControlSignal& cs ) { cs = pilot_control_signal; }
    
    bool ShouldClose() {
        return WindowShouldClose();
    }
    
    void Close() {
        CloseWindow();
    }
    
private:
    void DrawAircraft(const State& state)
    {
        Vector3 pos = {(float)state.x, (float)-state.z, (float)state.y};
        
        // Create rotation matrix from Euler angles
        Matrix rotation = MatrixIdentity();
        /*
        rotation = MatrixMultiply(rotation, MatrixRotateY((float)state.psi));    // Yaw
        rotation = MatrixMultiply(rotation, MatrixRotateX((float)state.theta));  // Pitch
        rotation = MatrixMultiply(rotation, MatrixRotateZ((float)state.phi));    // Roll
        */
        rotation = MatrixMultiply(rotation, MatrixRotateX((float)state.phi));    // Roll
        rotation = MatrixMultiply(rotation, MatrixRotateZ((float)state.theta));  // Pitch
        rotation = MatrixMultiply(rotation, MatrixRotateY((float)-state.psi));    // Yaw
        
        // Fuselage (elongated box)
        Vector3 fuselageSize = {8.0f, 1.5f, 1.5f};
        DrawCubeRotated(pos, rotation, fuselageSize, aircraftColor);
        
        // Wings (flat boxes)
        Vector3 wingOffset = Vector3Transform(Vector3{-0.8f, -0.1f, 0}, rotation);
        Vector3 wingPos = Vector3Add(pos, wingOffset);
        Vector3 wingSize = {1.8f, 0.2f, 10.0f};
        DrawCubeRotated(wingPos, rotation, wingSize, DARKBLUE);
        
        // Tail (vertical stabilizer)
        Vector3 tailOffset = Vector3Transform(Vector3{-4.5f, 1.5f, 0}, rotation);
        Vector3 tailPos = Vector3Add(pos, tailOffset);
        Vector3 tailSize = {1.0f, 3.0f, 0.2f};
        DrawCubeRotated(tailPos, rotation, tailSize, PURPLE);
        
        // Horizontal stabilizer
        Vector3 hstabOffset = Vector3Transform(Vector3{-4.5f, -0.3f, 0}, rotation);
        Vector3 hstabPos = Vector3Add(pos, hstabOffset);
        Vector3 hstabSize = {0.8f, 0.2f, 4.0f};
        DrawCubeRotated(hstabPos, rotation, hstabSize, BLUE);
        
        // Draw body axes
        Vector3 xAxis = Vector3Transform(Vector3{10, 0, 0}, rotation);
        Vector3 yAxis = Vector3Transform(Vector3{0, 0, 5}, rotation);
        Vector3 zAxis = Vector3Transform(Vector3{0, 5, 0}, rotation);
        
        DrawLine3D(pos, Vector3Add(pos, xAxis), RED);    // X - forward
        DrawLine3D(pos, Vector3Add(pos, zAxis), GREEN);  // Y - right
        DrawLine3D(pos, Vector3Add(pos, yAxis), BLUE);   // Z - down
    }
    
    void DrawCubeRotated(Vector3 position, Matrix rotation, Vector3 size, Color color)
    {
        // Draw a cube with custom rotation
        Vector3 vertices[8] = {
            {-size.x/2, -size.y/2, -size.z/2},
            { size.x/2, -size.y/2, -size.z/2},
            { size.x/2,  size.y/2, -size.z/2},
            {-size.x/2,  size.y/2, -size.z/2},
            {-size.x/2, -size.y/2,  size.z/2},
            { size.x/2, -size.y/2,  size.z/2},
            { size.x/2,  size.y/2,  size.z/2},
            {-size.x/2,  size.y/2,  size.z/2}
        };
        
        // Transform vertices
        for (int i = 0; i < 8; i++) {
            vertices[i] = Vector3Transform(vertices[i], rotation);
            vertices[i] = Vector3Add(vertices[i], position);
        }
        
        // Draw edges (wireframe for better visibility)
        DrawLine3D(vertices[0], vertices[1], color);
        DrawLine3D(vertices[1], vertices[2], color);
        DrawLine3D(vertices[2], vertices[3], color);
        DrawLine3D(vertices[3], vertices[0], color);
        
        DrawLine3D(vertices[4], vertices[5], color);
        DrawLine3D(vertices[5], vertices[6], color);
        DrawLine3D(vertices[6], vertices[7], color);
        DrawLine3D(vertices[7], vertices[4], color);
        
        DrawLine3D(vertices[0], vertices[4], color);
        DrawLine3D(vertices[1], vertices[5], color);
        DrawLine3D(vertices[2], vertices[6], color);
        DrawLine3D(vertices[3], vertices[7], color);
    }
    
    void DrawVelocityVector(const State& state)
    {
        Vector3 pos = {(float)state.x, (float)-state.z, (float)state.y};
        
        // Velocity in NED frame
        double V = sqrt(state.u*state.u + state.v*state.v + state.w*state.w);
        if (V < 0.1) return;
        
        // Transform velocity to world frame
        float psi = (float)state.psi;       // yaw
        float theta = (float)state.theta;   // pitch
        float phi = (float)state.phi;       // roll
        
        Matrix rotation = MatrixIdentity();
        /*
        rotation = MatrixMultiply(rotation, MatrixRotateY(psi));
        rotation = MatrixMultiply(rotation, MatrixRotateX(theta));
        rotation = MatrixMultiply(rotation, MatrixRotateZ(phi));
        */
        rotation = MatrixMultiply(rotation, MatrixRotateX(phi));    // Roll
        rotation = MatrixMultiply(rotation, MatrixRotateZ(theta));  // Pitch
        rotation = MatrixMultiply(rotation, MatrixRotateY(-psi));    // Yaw
        
        Vector3 velBody = {(float)state.u, (float)state.w, (float)state.v};
        //Vector3 velBody = {(float)state.u, (float)-state.w, (float)state.v};
        Vector3 velWorld = Vector3Transform(velBody, rotation);
        velWorld = Vector3Scale(velWorld, 0.5f);  // Scale for visibility
        
        Vector3 velEnd = Vector3Add(pos, velWorld);
        DrawLine3D(pos, velEnd, MAGENTA);
        DrawSphere(velEnd, 0.5f, MAGENTA);
    }
    
    void DrawHUD(const State& state, const ControlSignal& control_s)
    {
        int y = 10;
        int lineHeight = 20;
        
        // Calculate derived values
        double V = sqrt(state.u*state.u + state.v*state.v + state.w*state.w);
        double alpha = atan2(state.w, state.u) * 180.0 / PI;
        double beta = 0;
        if (V > 0.1) {
            beta = asin(Clamp(state.v / V, -1.0, 1.0)) * 180.0 / PI;
        }

        // Background
        DrawRectangle(5, 5, 445, 280, Fade(BLACK, 0.3f));
        
        // Position
        DrawTextFormatted(10, y, 20, WHITE, "Position (NED):");
        y += lineHeight;
        DrawTextFormatted(20, y, 16, LIGHTGRAY, "X: %.1f m\t Y: %.1f m\t Alt: %.1f m", 
                         state.x, state.y, -state.z);
        y += lineHeight + 5;
        
        // Velocity
        DrawTextFormatted(10, y, 20, WHITE, "Velocity:");
        y += lineHeight;
        DrawTextFormatted(20, y, 16, LIGHTGRAY, "u: %.1f\t v: %.1f\t w: %.1f\t |V|: %.1f m/s", 
                         state.u, state.v, state.w, V);
        y += lineHeight + 5;
        
        // Attitude
        DrawTextFormatted(10, y, 20, WHITE, "Attitude (deg):");
        y += lineHeight;
        DrawTextFormatted(20, y, 16, LIGHTGRAY, "Roll: %.1f\t Pitch: %.1f\t Yaw: %.1f", 
                         state.phi * 180/PI, state.theta * 180/PI, state.psi * 180/PI);
        y += lineHeight + 5;
        
        // Rates
        DrawTextFormatted(10, y, 20, WHITE, "Rates (deg/s):");
        y += lineHeight;
        DrawTextFormatted(20, y, 16, LIGHTGRAY, "p: %.1f\t q: %.1f\t r: %.1f", 
                         state.p * 180/PI, state.q * 180/PI, state.r * 180/PI);
        y += lineHeight + 5;
        
        // Control inputs
        DrawTextFormatted(10, y, 20, WHITE, "Control Signal:");
        y += lineHeight;
        DrawTextFormatted(20, y, 16, LIGHTGRAY, "thr: %.1f\t elev: %.1f\t aile: %.1f\t rudd: %.1f", 
                         control_s.throttle, control_s.elevator, control_s.aileron, control_s.rudder);
        y += lineHeight + 5;
        
        // Flow angles
        DrawTextFormatted(10, y, 20, WHITE, "Flow Angles:");
        y += lineHeight;
        DrawTextFormatted(20, y, 16, LIGHTGRAY, "Alpha: %.2f°\t Beta: %.2f°", alpha, beta);
        
        // Draw artificial horizon (right side)
        int horizonX = screenWidth - 220;
        int horizonY = 100;
        int horizonSize = 200;
        DrawArtificialHorizon(horizonX, horizonY, horizonSize, state.phi, state.theta);
    }
    
    void DrawArtificialHorizon(int x, int y, int size, double roll, double pitch)
    {
        // Background
        DrawRectangle(x - size/2, y - size/2, size, size, Color{40, 40, 40, 200});
        DrawRectangleLines(x - size/2, y - size/2, size, size, WHITE);
        
        // Sky and ground (simplified)
        int pitchOffset = (int)(pitch * 180.0 / PI * 3.0);  // Scale pitch to pixels
        
        // Rotate around center
        float rollRad = (float)roll;
        
        // Draw horizon line
        Vector2 horizonLeft = {x - size/2.0f, static_cast<float>(y + pitchOffset)};
        Vector2 horizonRight = {x + size/2.0f, static_cast<float>(y + pitchOffset)};
        
        // Rotate points around center
        Vector2 center = {(float)x, (float)y};
        horizonLeft = RotatePointAroundCenter(horizonLeft, center, -rollRad);
        horizonRight = RotatePointAroundCenter(horizonRight, center, -rollRad);
        
        DrawLineEx(horizonLeft, horizonRight, 3, YELLOW);
        
        // Aircraft symbol (fixed in center)
        DrawLine(x - 30, y, x - 10, y, RED);
        DrawLine(x + 10, y, x + 30, y, RED);
        DrawCircle(x, y, 3, RED);
        
        // Labels
        DrawText("AH", x - size/2 + 5, y - size/2 + 5, 16, WHITE);
    }
    
    Vector2 RotatePointAroundCenter(Vector2 point, Vector2 center, float angle)
    {
        float s = sin(angle);
        float c = cos(angle);
        
        point.x -= center.x;
        point.y -= center.y;
        
        float xnew = point.x * c - point.y * s;
        float ynew = point.x * s + point.y * c;
        
        point.x = xnew + center.x;
        point.y = ynew + center.y;
        
        return point;
    }
    
    void DrawControlsHelp()
    {
        int y = screenHeight - 140;
        DrawRectangle(10, y - 5, 260, 135, Color{0, 0, 0, 150});
        
        DrawText("Controls:", 15, y, 16, YELLOW);
        y += 20;
        DrawText("G - Toggle Grid", 15, y, 14, WHITE); y += 18;
        DrawText("A - Toggle Axes", 15, y, 14, WHITE); y += 18;
        DrawText("T - Toggle Trail", 15, y, 14, WHITE); y += 18;
        DrawText("V - Toggle Velocity", 15, y, 14, WHITE); y += 18;
        DrawText("H - Toggle HUD", 15, y, 14, WHITE); y += 18;
        DrawText("F - Toggle Follow", 15, y, 14, WHITE); y += 18;
        DrawText("+/- - Zoom", 15, y, 14, WHITE);
    }
    
    void DrawTextFormatted(int x, int y, int size, Color color, const char* format, ...)
    {
        char buffer[256];
        va_list args;
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        DrawText(buffer, x, y, size, color);
    }
};

#endif // FLIGHT_VISUALIZER_H
