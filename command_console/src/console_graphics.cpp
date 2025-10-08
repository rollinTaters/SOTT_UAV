/*
	MIT License

	Copyright (c) 2025 rollinTaters

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:
	
	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.
	
	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.
*/


#include "console_graphics.hpp"
#include "gauge.hpp"
namespace cg {

    static bool _initialized = false;
    
    Font arial_font = LoadFont("./assets/fonts/arial.ttf");

    void InitWindowSafe(int width, int height, const char* title) {
        if (_initialized) return;
        InitWindow(width, height, title);
        SetTargetFPS(60);
        _initialized = true;  
        gauge_compass.init();
        gauge_adi.init();
        steering_wheel.init();
    }

    bool isInitialized() {
        return _initialized;
    }

    void createPanel(Vector2 pos, Vector2 size , Color color , const char* title){
        DrawTextEx(arial_font, title, {pos.x, pos.y - 25}, 25, 1 ,RED);
        DrawRectangle(pos.x, pos.y , size.x, size.y, color);
    }



    Gauge gauge_temp      (Gauge::type_temperature, {565, 65}, 180.0f);
    Gauge gauge_amp       (Gauge::type_amp,         {760, 65}, 180.0f);
    Gauge gauge_amp2      (Gauge::type_amp,         {760,284}, 180.0f);
    Gauge gauge_temp2     (Gauge::type_temperature, {565,284}, 180.0f);
    Gauge gauge_speed     (Gauge::type_speedometer, {250, 85}, 200.0f);
    Gauge gauge_compass   (Gauge::type_compass,     {400,650}, 200.0f);
    Gauge gauge_tachometer(Gauge::type_tachometer,  { 50, 85}, 200.0f);
    Gauge gauge_battery   (Gauge::type_battery,     { 30, 10}, 100.0f);
    Gauge gauge_signal    (Gauge::type_signal,      {170, 60}, 100.0f);
    Adi   gauge_adi       ({150, 500},  300.0f);
    UserInput input;
    Gauge steering_wheel  (Gauge::type_steering_wheel, {665,620}, 200.0f);

    void renderGauges()
    {
        gauge_adi.render();
        createPanel({550,50},{400,450},DARKGRAY,"Motor Panel");
        gauge_amp2.render();
        gauge_temp2.render();
        gauge_amp.render();
        gauge_temp.render();
        gauge_speed.render();
        gauge_tachometer.render();
        gauge_compass.render();
        gauge_battery.render();
        gauge_signal.render();
        steering_wheel.render();
    }

    // -- DEBUG --
    /*                   __
                        // \
                        \\_/ //
      ''-.._.-''-.._.. -(||)(')
                         '''
    */
   
    void DEBUG_gauge_test()
    {
        static float g_values[12] = {0};

       // nudge value towards a random direction
        for( float &g : g_values )
        {
            g += ((rand()/float(RAND_MAX))-0.5f)*0.05f;
            g = std::max( 0.f, std::min( g, 1.f ));
        }

        gauge_amp.       updateProportionalVal( g_values[0] );
        gauge_amp2.      updateProportionalVal( g_values[1] );
        gauge_temp.      updateProportionalVal( g_values[2] );
        gauge_temp2.     updateProportionalVal( g_values[3] );
        gauge_battery.   updateProportionalVal( g_values[4] );
        gauge_adi.       updateRollVal_prop   ( g_values[5] );
        gauge_adi.       updatePitchVal_prop  ( g_values[6] );
        gauge_speed.     updateProportionalVal( g_values[7] );
        gauge_compass.   updateProportionalVal( g_values[8] );
        gauge_signal.    updateProportionalVal( g_values[9] );
        gauge_tachometer.updateProportionalVal( g_values[10] );
        steering_wheel.updateProportionalVal(g_values[11]);
    }
    // -- END OF DEBUG --
}
    
