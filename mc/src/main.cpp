#include <opencv2/opencv.hpp>
#include "raylib.h"
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

int main() {
    // 1. Initialize OpenCV Camera
    VideoCapture cap(0);
    cap.set(CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CAP_PROP_FRAME_HEIGHT, 480);
    if (!cap.isOpened()) return -1;

    // 2. Initialize Raylib Window
    InitWindow(640, 480*2, "Robot Vision - Raylib Display");
    SetTargetFPS(30);

    // Create a Raylib Image and Texture that we will update every frame
    Image rayImage = GenImageColor(640, 480, BLANK);
    rayImage.format = PIXELFORMAT_UNCOMPRESSED_R8G8B8; // Standard RGB
    Texture2D texture1 = LoadTextureFromImage(rayImage);
    Texture2D texture2 = LoadTextureFromImage(rayImage);

    Mat frame, gray, processed;
    Mat gui_frame1, gui_frame2;   // for gui display

    while (!WindowShouldClose()) {
        cap >> frame;
        if (frame.empty()) break;

        // --- OPENCV PROCESSING SECTION ---
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        GaussianBlur(gray, gray, Size(5, 5), 0);
        Canny(gray, processed, 50, 150);

        vector<vector<Point>> contours;
        findContours(processed, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        // --- CONVERT CV::MAT TO RAYLIB TEXTURE ---
        // Convert BGR (OpenCV) to RGB (Raylib)
        cvtColor(frame, gui_frame1, COLOR_BGR2RGB);
        //cvtColor(gray, gui_frame2, COLOR_GRAY2RGB);
        cvtColor(processed, gui_frame2, COLOR_GRAY2RGB);
        
        // Update the texture directly on the GPU
        UpdateTexture(texture1, gui_frame1.data);
        UpdateTexture(texture2, gui_frame2.data);

        // --- RAYLIB DRAWING SECTION ---
        BeginDrawing();
            ClearBackground(BLACK);
            
            // Draw the camera feed
            DrawTexture(texture1, 0, 0, WHITE);
            DrawTexture(texture2, 0, 480, WHITE);

            // Draw shape labels based on OpenCV results
            for (const auto& contour : contours) {
                if (contourArea(contour) < 100) continue;

                vector<Point> approx;
                approxPolyDP(contour, approx, 0.04 * arcLength(contour, true), true);

                // Draw geometry using Raylib primitives
                int x = approx[0].x;
                int y = approx[0].y;

                if (approx.size() == 3) {
                    DrawText("TRIANGLE", x, y - 20, 20, YELLOW);
                    DrawCircle(x, y, 5, YELLOW);
                } else if (approx.size() == 4) {
                    DrawText("SQUARE", x, y - 20, 20, GREEN);
                    DrawCircle(x, y, 5, GREEN);
                } else if (approx.size() > 7) {
                    DrawText("CIRCLE", x, y - 20, 20, RED);
                    DrawCircle(x, y, 5, RED);
                }
            }

            DrawFPS(10, 10);
        EndDrawing();
    }

    UnloadTexture(texture1);
    UnloadTexture(texture2);
    CloseWindow();
    return 0;
}
