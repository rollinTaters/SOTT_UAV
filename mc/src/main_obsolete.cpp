#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

int main() {
    // 0 is usually the default USB webcam
    VideoCapture cap(0);
    if (!cap.isOpened()) {
        cerr << "Error: Could not open webcam." << endl;
        return -1;
    }

    // Set lower resolution for better performance on Pi Zero 2
    cap.set(CAP_PROP_FRAME_WIDTH, 320);
    cap.set(CAP_PROP_FRAME_HEIGHT, 240);

    Mat frame, gray, blurred, thresholded;

    cout << "Starting Detection... Press any key to exit (if window is open)." << endl;

    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        // 1. Pre-processing
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        GaussianBlur(gray, blurred, Size(5, 5), 0);
        Canny(blurred, thresholded, 50, 150);

        // 2. Find Contours
        vector<vector<Point>> contours;
        findContours(thresholded, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        for (const auto& contour : contours) {
            // Filter out small noise
            if (contourArea(contour) < 500) continue;

            // 3. Approximate Shape
            vector<Point> approx;
            double peri = arcLength(contour, true);
            approxPolyDP(contour, approx, 0.04 * peri, true);

            // 4. Identify and Print
            int vertices = approx.size();
            string shapeName = "Unknown";

            if (vertices == 3) shapeName = "Triangle";
            else if (vertices == 4) shapeName = "Square/Rectangle";
            else if (vertices > 7) shapeName = "Circle/Oval";

            if (shapeName != "Unknown") {
                cout << "Detected: " << shapeName << " | Vertices: " << vertices << endl;
            }
        }

        // Optional: If you are running with a monitor attached
        imshow("Feed", frame);
        if (waitKey(30) >= 0) break;
    }

    return 0;
}
