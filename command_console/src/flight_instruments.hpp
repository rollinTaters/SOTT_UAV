#include "raylib.h"
#include "rlgl.h"
#include "raymath.h"

class ADI {
private:
    Vector2 m_center;
    float radius;
    float pitch; // In degrees
    float roll;  // In degrees

public:
    ADI(Vector2 pos, float r) : m_center(pos), radius(r), pitch(0), roll(0) {}

    void update(float newPitch, float newRoll)
    {
        pitch = newPitch;
        roll = newRoll;
    }

    void render( Vector2 offset )
    {
        Vector2 center = m_center + offset;

        // 1. Draw the Outer Ring/Housing
        DrawCircleV(center, radius + 5, DARKGRAY);

        // Use Scissoring to keep the horizon inside the circle
        BeginScissorMode(center.x - radius, center.y - radius, radius * 2, radius * 2);

        // 2. Draw the Rotating Horizon
        // We move the "center" of the horizon based on pitch
        // 1 degree of pitch = 5 pixels (arbitrary scale)
        float pitchOffset = pitch * 5.0f;

        rlPushMatrix();
            rlTranslatef(center.x, center.y, 0);
            rlRotatef(-roll, 0, 0, 1); // Rotate for Roll
            rlTranslatef(0, pitchOffset, 0); // Translate for Pitch

            // Sky (Blue)
            DrawRectangle(-radius * 2, -radius * 4, radius * 4, radius * 4, SKYBLUE);
            // Ground (Brown)
            DrawRectangle(-radius * 2, 0, radius * 4, radius * 4, BROWN);
            // Horizon Line
            DrawLineEx({-radius * 2, 0}, {radius * 2, 0}, 3.0f, WHITE);

            // 3. Pitch Ladder
            for (int i = -90; i <= 90; i += 5) {
                if(i == 0) continue;
                float yPos = -i * 5.0f;
                float width = (i % 10 == 0) ? 70.0f : 20.0f;
                DrawLineEx({-width, yPos}, {width, yPos}, 2.0f, WHITE);
                if(i % 10 == 0) DrawText(TextFormat("%d",i), width + 3, yPos, 15, WHITE);
            }
        rlPopMatrix();

        EndScissorMode();

        // 4. Fixed Aircraft Symbol (Drawn after Scissor)
        DrawLineEx({center.x - 60, center.y}, {center.x - 20, center.y}, 4.0f, YELLOW);
        DrawLineEx({center.x + 20, center.y}, {center.x + 60, center.y}, 4.0f, YELLOW);
        DrawCircle(center.x, center.y, 4, YELLOW);
        
        // Decorative Border
        DrawCircleLines(center.x, center.y, radius, WHITE);
    }
};

class HeadingIndicator {
private:
    Vector2 m_center;
    float radius;
    float heading; 

public:
    HeadingIndicator(Vector2 pos, float r) : m_center(pos), radius(r), heading(0) {}

    void update(float newHeading) {
        // Keep heading within 0-359 range
        heading = fmodf(newHeading, 360.0f);
        if (heading < 0) heading += 360.0f;
    }

    void render(Vector2 offset) {
        Vector2 center = m_center + offset;

        // 1. Draw Background
        DrawCircleV(center, radius, ColorAlpha(BLACK, 0.6f));
        DrawCircleLines(center.x, center.y, radius, DARKGRAY);

        // 2. Draw Rotating Ticks
        rlPushMatrix();
            rlTranslatef(center.x, center.y, 0);
            rlRotatef(-heading, 0, 0, 1); 

            for (int i = 0; i < 360; i += 10) {
                float angleRad = (i - 90) * DEG2RAD;
                Vector2 inner = { cosf(angleRad) * (radius - 12), sinf(angleRad) * (radius - 12) };
                Vector2 outer = { cosf(angleRad) * radius, sinf(angleRad) * radius };
                
                float thickness = (i % 30 == 0) ? 3.0f : 1.5f;
                DrawLineEx(inner, outer, thickness, LIGHTGRAY);
            }
        rlPopMatrix();

        // 3. Draw Upright Labels
        for (int i = 0; i < 360; i += 30) {
            float drawAngle = (i - heading - 90) * DEG2RAD;
            
            const char* text;
            Color textColor = WHITE;
            int fontSize = 18;
            bool isCardinal = false;

            if (i == 0) { text = "N"; isCardinal = true; }
            else if (i == 90) { text = "E"; isCardinal = true; }
            else if (i == 180) { text = "S"; isCardinal = true; }
            else if (i == 270) { text = "W"; isCardinal = true; }
            else { text = TextFormat("%d", i / 10); }

            if (isCardinal) {
                fontSize = 22;
                textColor = Fade(YELLOW,0.7f);
            }

            int textWidth = MeasureText(text, fontSize);
            Vector2 textPos = {
                center.x + cosf(drawAngle) * (radius - 35),
                center.y + sinf(drawAngle) * (radius - 35)
            };

            DrawText(text, textPos.x - textWidth / 2, textPos.y - fontSize / 2, fontSize, textColor);
        }

        // 4. Digital Readout Box (At the Top)
        Rectangle readoutBox = { center.x - 30, center.y - radius - 45, 60, 30 };
        DrawRectangleRec(readoutBox, BLACK);
        DrawRectangleLinesEx(readoutBox, 2, WHITE);
        
        const char* headTxt = TextFormat("%03d", (int)heading);
        int txtW = MeasureText(headTxt, 20);
        DrawText(headTxt, readoutBox.x + (readoutBox.width/2 - txtW/2), readoutBox.y + 5, 20, YELLOW);

        // 5. Fixed Lubber Line (Pointer)
        // This is a bright triangle pointing down at the current heading
        Vector2 p1 = { center.x, center.y - radius + 5 };       // Tip
        Vector2 p2 = { center.x - 12, center.y - radius - 15 }; // Left base
        Vector2 p3 = { center.x + 12, center.y - radius - 15 }; // Right base
        
        DrawTriangle(p1, p2, p3, RED); // Using a standard triangle
        DrawLineEx(p1, {center.x, center.y - radius + 25}, 2.0f, RED); // Tail of the pointer
    }
};

class Altimeter {
private:
    Rectangle m_bounds;
    float altitude;

public:
    Altimeter(Rectangle rect) : m_bounds(rect), altitude(0) {}

    void update(float newAltitude)
    {
        altitude = newAltitude;
    }

    void render( Vector2 offset )
    {
        Rectangle bounds = m_bounds;
        bounds.x += offset.x;
        bounds.y += offset.y;

        // Background
        DrawRectangleRec(bounds, ColorAlpha(BLACK, 0.6f));
        DrawRectangleLinesEx(bounds, 2, LIGHTGRAY);

        BeginScissorMode(bounds.x, bounds.y, bounds.width, bounds.height);

            float pixelsPerFoot = 0.5f; // Scale factor
            int startAlt = ((int)altitude / 100) * 100 - 500;
            int endAlt = ((int)altitude / 100) * 100 + 500;

            for (int i = startAlt; i <= endAlt; i += 100) {
                // Calculate Y position relative to current altitude
                float yOffset = (altitude - i) * pixelsPerFoot;
                float yPos = (bounds.y + bounds.height / 2) + yOffset;

                DrawLine(bounds.x + bounds.width - 15, yPos, bounds.x + bounds.width, yPos, WHITE);
                DrawText(TextFormat("%d", i), bounds.x + 5, yPos - 10, 20, WHITE);
            }

        EndScissorMode();

        // Current Altitude Readout Box (Center Overlay)
        Rectangle readout = { bounds.x - 10, bounds.y + bounds.height/2 - 15, bounds.width + 10, 30 };
        DrawRectangleRec(readout, DARKGRAY);
        DrawRectangleLinesEx(readout, 2, YELLOW);
        DrawText(TextFormat("%0.0f", altitude), readout.x + 15, readout.y + 5, 20, YELLOW);
    }
};

class AirspeedIndicator {
private:
    Rectangle m_bounds;
    float speed;

public:
    AirspeedIndicator(Rectangle rect) : m_bounds(rect), speed(0) {}

    void update(float newSpeed) {
        speed = (newSpeed < 0) ? 0 : newSpeed; // Airspeed can't be negative
    }

    void render( Vector2 offset ) {
        Rectangle bounds(m_bounds);
        bounds.x += offset.x;
        bounds.y += offset.y;

        DrawRectangleRec(bounds, ColorAlpha(BLACK, 0.6f));
        DrawRectangleLinesEx(bounds, 2, LIGHTGRAY);

        BeginScissorMode(bounds.x, bounds.y, bounds.width, bounds.height);
            float pixelsPerKnot = 2.0f; 
            int startSpd = ((int)speed / 20) * 20 - 100;
            int endSpd = ((int)speed / 20) * 20 + 100;

            for (int i = startSpd; i <= endSpd; i += 20) {
                if (i < 0) continue;

                float yOffset = (speed - i) * pixelsPerKnot;
                float yPos = (bounds.y + bounds.height / 2) + yOffset;

                // Speed Tick marks
                DrawLine(bounds.x, yPos, bounds.x + 15, yPos, WHITE);
                DrawText(TextFormat("%d", i), bounds.x + 20, yPos - 10, 20, WHITE);
                
                // Example Color Arc (e.g., Redline at 250 knots)
                if (i >= 250) DrawRectangle(bounds.x + bounds.width - 5, yPos, 5, 40, RED);
            }
        EndScissorMode();

        // Current Speed Readout
        Rectangle readout = { bounds.x, bounds.y + bounds.height/2 - 15, bounds.width + 10, 30 };
        DrawRectangleRec(readout, DARKGRAY);
        DrawRectangleLinesEx(readout, 2, WHITE);
        DrawText(TextFormat("%0.0f", speed), readout.x + 10, readout.y + 5, 20, WHITE);
    }
};

class VerticalSpeedIndicator {
private:
    Vector2 m_position;
    float height;
    float vsi; // Feet per minute

public:
    VerticalSpeedIndicator(Vector2 pos, float h) : m_position(pos), height(h), vsi(0) {}

    void update(float verticalRate) {
        vsi = verticalRate;
    }

    void render( Vector2 offset ) {
        Vector2 position = m_position + offset;

        // Vertical scale line
        DrawLineEx(position, {position.x, position.y + height}, 3, LIGHTGRAY);
        
        // Zero mark
        float centerY = position.y + height / 2;
        DrawLineEx({position.x - 10, centerY}, {position.x + 10, centerY}, 2, WHITE);
        DrawText("0", position.x + 15, centerY - 10, 15, WHITE);

        // VSI Needle/Indicator (Current Rate)
        // Scale: 2000 fpm = full deflection
        float deflection = (vsi / 2000.0f) * (height / 2);
        // Clamp deflection to height
        if (deflection > height/2) deflection = height/2;
        if (deflection < -height/2) deflection = -height/2;

        float targetY = centerY - deflection; // Negative is up in screen space

        DrawTriangle({position.x + 5, targetY}, 
                     {position.x + 25, targetY - 10}, 
                     {position.x + 25, targetY + 10}, YELLOW);
        
        DrawText(TextFormat("%0.0f", vsi), position.x + 30, targetY - 7, 15, YELLOW);
    }
};
