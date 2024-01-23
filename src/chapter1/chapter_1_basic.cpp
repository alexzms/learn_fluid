#include <iostream>
#include "cmath"
#include "memory"
#include "string"
#include "thread"
#include "jet/jet.h"
// kbhit
#include <conio.h>

#define HEIGHT_BUFFER_SIZE 128
#define M_PI 3.14159265358979323846

const char* const kVisualGrayScale = " .:-=+*#%@";
const size_t kVisualGrayScaleLength = strlen(kVisualGrayScale);
bool gui_running = true;

void visualize_height_field(const std::unique_ptr<double[]>& height_field) {
    std::string buffer(HEIGHT_BUFFER_SIZE, ' ');
    for (int i = 0; i != HEIGHT_BUFFER_SIZE; ++i) {
        int index = static_cast<int>(height_field[i] * (double)kVisualGrayScaleLength);
        if (index > kVisualGrayScaleLength - 1) index = (int)kVisualGrayScaleLength - 1;
        buffer[i] = kVisualGrayScale[index];
    }
    // clear previous line
    std::cout << "\r";
    // print current line
    std::cout << buffer;
    std::cout.flush();
}


void accumulate_wave_to_height_field(const double pos, const double wave_length,
                                     const double max_height, std::unique_ptr<double[]>& height_field) {
    const double quarter_wave_length = wave_length * 0.25;      // only render 1/2 of the wave
    const int start = static_cast<int>((pos - quarter_wave_length) * HEIGHT_BUFFER_SIZE);
    const int end = static_cast<int>((pos + quarter_wave_length) * HEIGHT_BUFFER_SIZE);
    for (int i = start; i != end; ++i) {
        int index = i;
        if (index < 0) index = -index - 1;                      // reflect to positive/negative
        else if (index > HEIGHT_BUFFER_SIZE - 1) index = 2 * HEIGHT_BUFFER_SIZE - index - 1;
        double distance = fabs((i + 0.5) / HEIGHT_BUFFER_SIZE - pos);   // use original i for value
        double val =
            max_height * 0.5 * (cos(fmin(distance * M_PI / quarter_wave_length, M_PI)) + 1.0);
        height_field[index] += val;
    }
}

void clear_height_field(std::unique_ptr<double[]>& height_field) {
    for (int i = 0; i != HEIGHT_BUFFER_SIZE; ++i) {
        height_field[i] = 0.0;
    }
}


void update_wave(const double interval, double& pos, double& speed) {
    pos += interval * speed;
    if (pos > 1.0) {                                            // boundary check
        speed = -speed;
        pos = 1.0 + interval * speed;
    } else if (pos < 0.0) {
        speed = -speed;
        pos = interval * speed;
    }
}

void process_input() {
    if (_kbhit()) {
        int key = _getch();
        if (key == 'q') {
            gui_running = false;
        }
    }
}

int main() {
    std::cout << "Chapter1: Hello Fluid Simulator" << std::endl;
                                                                // default as 0 initialization
    std::unique_ptr<double[]> height_field = std::make_unique<double[]>(HEIGHT_BUFFER_SIZE);

    double pos1 = 0.0;
    double pos2 = 1.0;
    double pos3 = 0.5;
    double speed1 = 1.0;
    double speed2 = -0.5;
    double speed3 = 0.2;
    double wave_length1 = 0.8;
    double wave_length2 = 1.2;
    double wave_length3 = 0.6;
    double max_height1 = 0.5;
    double max_height2 = 0.4;
    double max_height3 = 0.45;

    jet::Vector2D grid_size(64, 64);

    const unsigned int fps = 100;
    const double time_interval = 1.0 / fps;

    while (gui_running) {
        process_input();
        clear_height_field(height_field);
        update_wave(time_interval, pos1, speed1);
        update_wave(time_interval, pos2, speed2);
        update_wave(time_interval, pos3, speed3);
        accumulate_wave_to_height_field(pos1, wave_length1, max_height1, height_field);
        accumulate_wave_to_height_field(pos2, wave_length2, max_height2, height_field);
        accumulate_wave_to_height_field(pos3, wave_length3, max_height3, height_field);
        visualize_height_field(height_field);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000 / fps));
    }
    std::cout << std::endl;
    return 0;
}
