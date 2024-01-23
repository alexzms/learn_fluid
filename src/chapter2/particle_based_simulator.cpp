#include "jet/jet.h"
#include "glm/glm.hpp"
#include "iostream"
#include "visualime/visualime.h"
#include "functional"
#include "mutex"

glm::vec<3, unsigned char> mapVelocityToColor(const jet::Vector3D& vel, double normalize_factor = 5)
{
    double amplitude = vel.length() / normalize_factor;
    // faster, more red, slower, more blue
    return glm::vec<3, unsigned char>{255 * amplitude, 0, 0}
         + glm::vec<3, unsigned char>{0, 0, 255 * (1.0 - amplitude)};
}

void FirstSightOnParticleSystemSolver3() {
    using namespace jet;
    using namespace visualime;
    std::mutex modification_mutex;
    Plane3Ptr plane = std::make_shared<Plane3>(Vector3D(0, 1, 0), Vector3D());
    RigidBodyCollider3Ptr collider
        = std::make_shared<RigidBodyCollider3>(plane);
    ParticleSystemSolver3 solver;
    solver.setCollider(collider);
    solver.setDragCoefficient(0.0);
    solver.setRestitutionCoefficient(1.0);
    Frame frame;
    double timeIntervalInSeconds = 1.0 / 500.0;
    long long timeIntervalInMilliSeconds = 1;
    frame.timeIntervalInSeconds = timeIntervalInSeconds;

    ParticleSystemData3Ptr particles = solver.particleSystemData();

    particles->addParticle({0.5, 1, 0}, Vector3D{}, Vector3D{});
    GLsizei width = 800, height = 800;
    scene::scene2d scene{width, height, 1.0, false};
    std::vector<unsigned> circle_index;
    std::function<void(const glm::vec2&, double)> add_circle = [&] (const glm::vec2& pos, double r) -> void {
        modification_mutex.lock();
        particles->addParticle(
                {pos.x / width, 1.0f - pos.y / height, 0},
                Vector3D{}, Vector3D{}
                );
        circle_index.emplace_back(
    scene.add_circle_normalized(
            {176, 132, 169},
            glm::vec2{pos.x / width, 1.0f - pos.y / height},
            0.1, r)
        );
        modification_mutex.unlock();
    };
    std::function<void(const glm::vec2&)> left_click = [&](const glm::vec2& pos) { add_circle(pos, 0.01); };
    std::function<void(const glm::vec2&)> right_click = [&](const glm::vec2& pos) { add_circle(pos, 0.03); };
    scene.on_mouse_left_click = left_click;
    scene.on_mouse_right_click = right_click;

    for(auto& pos: particles->positions())
    {
        circle_index.emplace_back(
        scene.add_circle_normalized({176, 132, 169}, glm::vec2{pos.x, pos.y}, 0.1, 0.01)
        );
    }
    scene.launch(false);

    if (!scene.is_running())
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    double prev_energy = 9.8;
    std::vector<std::string> check_result{"False", "True"};
    int counter = 0;
    const int print_energy_counter = 300;
    while (true)
    {
        counter += 1;
        if (counter % print_energy_counter == 0)
        {
            double Ek = 0.5 * 1.0 * particles->velocities()[0].dot(particles->velocities()[0]);
            double Eg = 1.0 * 9.8 * particles->positions()[0].y;

            std::cout << "[info] Energy preservation check: " << "Ek + Eg(" << (Ek + Eg) << ") = previous("
                      << prev_energy << ") -> " << check_result[fabs(Ek + Eg - prev_energy) < 1e-6] << std::endl;
            prev_energy = Ek + Eg;
        }
        modification_mutex.lock();
        solver.update(frame);
        for (size_t i = 0; i != particles->positions().size(); ++i)
        {
            scene.change_primitive_position_normalized(
                    circle_index[i],
                    {particles->positions()[i].x, particles->positions()[i].y}
                    );
            scene.change_primitive_color(circle_index[i],
                                         mapVelocityToColor(particles->velocities()[i]));
        }
        scene.refresh();
        frame.advance();
        modification_mutex.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(timeIntervalInMilliSeconds));
        if (!scene.is_running()) {
            break;
        }
    }
    std::cout << "FirstSightOnParticleSystemSolver3, sighted" << std::endl;
}

int main() {
    jet::Logging::mute();
    FirstSightOnParticleSystemSolver3();
}


