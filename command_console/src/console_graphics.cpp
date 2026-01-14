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
namespace cg {

    static bool _initialized = false;
    Font arial_font;
    UserInput input;

    FlightInstrumentsPanel FI_panel;
    EngineeringPanel E_panel;

    Gauge gauge_speed     (Gauge::type_speedometer, {450, 500}, 200.0f);
    Gauge gauge_tachometer(Gauge::type_tachometer,  {240, 500}, 200.0f);


    void InitWindowSafe(int width, int height, const char* title)
    {
        if (_initialized) return;
        InitWindow(width, height, title);
        SetTargetFPS(60);
        arial_font = LoadFont("./assets/fonts/arial.ttf");
        _initialized = true;  
    }

    bool isInitialized()
    {
        return _initialized;
    }

    void createPanel(Vector2 pos, Vector2 size , Color color , const char* title)
    {
        DrawRectangle(pos.x, pos.y , size.x, size.y, color);
        DrawTextEx(arial_font, title, {pos.x, pos.y + 5}, 25, 1 ,RED);
    }

    void renderGauges()
    {
        FI_panel.render();
        E_panel.render();

        gauge_speed.render();
        gauge_tachometer.render();
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

        gauge_speed.     updateProportionalVal( g_values[1] );
        gauge_tachometer.updateProportionalVal( g_values[3] );
    }
    // -- END OF DEBUG --
}
    
