/*
 * This file is to test the animation function of the jet library
 */

#include "jet/jet.h"
#include "iostream"
#include "visualime/library.h"
#include "string"
#include "cmath"
#include "fstream"
#include "chrono"

const char* result_file = "./result/animation_fun/";


class SineAnimation final : public jet::Animation {
public:
    SineAnimation() = default;
    double x = 0.0;
private:
    void onUpdate(const jet::Frame& frame) override {
        x = std::sin(frame.timeInSeconds());
    }
};

static std::random_device rd;
static std::mt19937 mt(rd());
static std::uniform_real_distribution<double> dist(0.0, 1.0);

glm::vec3 random_rancolor() {
    return {static_cast<unsigned char>(dist(mt) * 255),
            static_cast<unsigned char>(dist(mt) * 255),
            static_cast<unsigned char>(dist(mt) * 255)};
}

void test_sine_animation_visualime() {
    using namespace jet;
    using namespace visualime;

    SineAnimation sine_animation;                                               // animation and visualisation
    Frame frame;
    test_scene2d scene{800, 800, 0.1};
    size_t circle_index = scene.add_circle({176, 132, 169}, {0, 0, 0.1}, 0.03);
//    size_t circle_index2 = scene.add_circle_normalized({176, 132, 169}, {0.5, 0.5, 0.1}, 0.03);
    scene.launch();
    const unsigned int FULL_TIMESTEP = 1024;
    for (int i = 0; i != FULL_TIMESTEP; ++i) {
        auto start = std::chrono::high_resolution_clock::now();
        if (!scene.get_run_thread().joinable()) break;
        sine_animation.update(frame);
        scene.change_primitive_position_normalized(
                circle_index, {(float)i / FULL_TIMESTEP, (sine_animation.x + 1) / 2, 0.1});
        if (i % 4 == 0) {
            scene.add_circle_normalized(random_rancolor(),
                             {(float)i / FULL_TIMESTEP, (sine_animation.x + 1) / 2, 0.1}, 0.01);
        }
        frame.advance();
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        std::this_thread::sleep_for(std::chrono::milliseconds(16) - elapsed);
    }
    if (scene.get_run_thread().joinable()) {
        scene.get_run_thread().join();
    }
    std::cout << "Sine animation finished" << std::endl;
}

void test_sine_animation() {
    using namespace jet;
    using namespace visualime;
    SineAnimation sine_animation;                                               // animation and visualisation
    Frame frame;
    std::ofstream file;
    std::string filename = std::string(result_file) + "sine_animation.txt";
    file.open(filename);
    if (!file) {
        std::cout << "Error opening file" << std::endl;
        exit(1);
    }
    for (int i = 0; i != 4096; ++i) {
        sine_animation.update(frame);
        file << "frame: " << frame.index << ", time: " << frame.timeInSeconds() << ", x: " << sine_animation.x << "\n";
        frame.advance();
    }
    file << std::endl;
    file.close();
    std::cout << "Sine animation written to " << filename << std::endl;
}

class SimpleMassSpringAnimation: public jet::PhysicsAnimation {
public:
    explicit SimpleMassSpringAnimation(size_t points_num, double mass = 1.0, double rest_length = 1.0,
                                       double stiffness = 1.0, double damping = 1.0):
            _points_num(points_num), _mass(mass), _edge_num(points_num - 1), _rest_length(rest_length),
            _stiffness(stiffness), _damping(damping) {
        positions.resize(points_num);                                // each point corresponds to 1 pos, vel, f
        velocities.resize(points_num);
        forces.resize(points_num);
        edges.resize(_edge_num);                                      // but only corresponds to num-1 edges

        for (size_t i = 0; i != points_num; ++i) {
            positions[i].x = static_cast<double>(i);
        }
        for (size_t i = 0; i != _edge_num; ++i) {
            edges[i] = Edge{i, i + 1};           // connect points in a row
        }
    }

    void changeGravity(const jet::Vector3D& gravity) {
        this->_gravity = gravity;
    }

protected:
    void onAdvanceTimeStep(double timeIntervalInSeconds) override {
        using namespace jet;
        for (size_t i = 0; i != _points_num; ++i) {
            forces[i] = _mass * _gravity;                                    // first apply gravity
        }
        for (size_t i = 0; i != _edge_num; ++i) {
            size_t index1 = edges[i].first_connect;                         // hooke's law: F = -k(dis-rest) * \vec{r}
            size_t index2 = edges[i].second_connect;

            Vector3D pos1 = positions[index1];
            Vector3D pos2 = positions[index2];

            Vector3D r = pos1 - pos2;                                       // \vec{r} should be unit direction
            auto distance = r.length();
            if (distance > 0.0) {                                           // if distance <= 0, NO hooke's law
                r /= distance;                                              // normalize \vec{r} here(prevent dis=0)
                Vector3D force = -_stiffness * (distance - _rest_length) * r;
                forces[index1] += force;                                    // point1 get force
                forces[index2] -= force;                                    // point2 get negative force(Newton's 2)
            }

            Vector3D vel1 = velocities[index1];                             // damping(阻尼), F_d = -c*(v1-v2)
            Vector3D vel2 = velocities[index2];
            Vector3D damping = _damping * (vel1 - vel2);
            forces[index1] += damping;
            forces[index2] -= damping;
        }

        for (size_t i = 0; i != _points_num; ++i) {
            Vector3D new_acc = forces[i] / _mass;                            // a = f / m
            Vector3D new_vel = {0, 0, 0};                                   // TODO
            Vector3D new_pos = {0, 0, 0};                                   // TODO

            velocities[i] = new_vel;                                        // this is copy-by-value, no worry
            positions[i] = new_pos;
        }

        // apply constrains..
    }

private:
    struct Edge {
        size_t first_connect;
        size_t second_connect;
    };

    std::vector<jet::Vector3D> positions;
    std::vector<jet::Vector3D> velocities;
    std::vector<jet::Vector3D> forces;
    std::vector<Edge> edges;
    jet::Vector3D _gravity{0.0, -9.8, 0.0};
    double _mass;
    double _stiffness;
    double _damping;
    double _rest_length;
    size_t _points_num;
    size_t _edge_num;
};

void test_physics_animation() {
//    jet::PhysicsAnimation ani;
}


int main() {
    jet::Logging::mute();
    test_sine_animation_visualime();
    return 0;
}