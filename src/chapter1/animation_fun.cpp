/*
 * This file is to test the animation function of the jet library
 */

#include <utility>

#include "jet/jet.h"
#include "iostream"
#undef VISUALIME_USE_CUDA
#include "visualime/visualime.h"
#include "string"
#include "cmath"
#include "fstream"
#include "chrono"
#include "thread"
#include "mutex"

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
    scene::scene2d scene{800, 800, 1.0};
    size_t circle_index = scene.add_circle_normalized({176, 132, 169}, {0.5, 0.5}, 0.1, 0.01);
//    size_t circle_index2 = scene.add_circle_normalized({176, 132, 169}, {0.5, 0.5, 0.1}, 0.03);
    scene.launch();
    const unsigned int FULL_TIMESTEP = 1024;
    for (int i = 0; i != FULL_TIMESTEP; ++i) {
        auto start = std::chrono::high_resolution_clock::now();
        if (!scene.get_run_thread().joinable()) break;
        sine_animation.update(frame);
        scene.change_primitive_position_normalized(
                circle_index, {(float)i / FULL_TIMESTEP, (sine_animation.x + 1) / 2});
        if (i % 4 == 0) {
            scene.add_circle_normalized(random_rancolor(),
                             {(float)i / FULL_TIMESTEP, (sine_animation.x + 1) / 2}, 0.1, 0.01);
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

class CustomVectorField3D final : public jet::VectorField3 {
public:
    [[nodiscard]] jet::Vector3D sample(const jet::Vector3D& x) const override {
        return {x.x, x.y, x.z};
    }
};

class SimpleMassSpringAnimation: public jet::PhysicsAnimation {
public:
    struct Edge {
        size_t first_connect;
        size_t second_connect;
    };
    struct PointConstraint{
        size_t index;
        jet::Vector3D fixed_pos;
        jet::Vector3D fixed_vel;
    };

    std::vector<jet::Vector3D> positions;
    std::vector<jet::Vector3D> velocities;
    std::vector<jet::Vector3D> forces;
    std::vector<PointConstraint> constraints;
    std::vector<Edge> edges;
    std::vector<double> rest_lengths;

    explicit SimpleMassSpringAnimation(size_t points_num,
                                       jet::VectorField3Ptr wind =
                                               std::make_shared<jet::ConstantVectorField3>(jet::Vector3D{1.0, 0.0, 0.0}),
                                       double mass = 1.0, double rest_length = 1.0, double floor_pos = 0.0,
                                       double stiffness = 1.0, double damping = 1.0, double air_friction = 1.0,
                                       double restitution = 0.1):

            _points_num(points_num), _mass(mass), _edge_num(points_num - 1), _rest_length(rest_length),
            _floor_pos(floor_pos), _restitution(restitution),
            _stiffness(stiffness), _damping(damping), _air_friction(air_friction), _wind(std::move(wind)) {

        build_default_system(points_num);
    }

    void update_number_of_points(size_t points_num) {
        _points_num = points_num;
        _edge_num = points_num - 1;
    }

    void build_two_pendulum_system() {
        this->change_parameters(
                0.1, 0.05, 0.0, 100000, 0.0, 0.0, 0.1
        );
        _points_num = 3;
        _edge_num = 2;
        positions.resize(3);                                // each point corresponds to 1 pos, vel, f
        velocities.resize(3);
        forces.resize(3);
        edges.resize(2);                                      // but only corresponds to num-1 edges
        rest_lengths.clear();

        positions[0] = {0.5, 0.5, 0.0};
        positions[1] = {0.7, 0.5, 0.0};
        positions[2] = {0.7, 0.7, 0.0};

        edges[0] = Edge{0, 1};
        edges[1] = Edge{1, 2};

        rest_lengths.push_back((positions[edges[0].first_connect] - positions[edges[0].second_connect]).length());
        rest_lengths.push_back((positions[edges[1].first_connect] - positions[edges[1].second_connect]).length());

        PointConstraint constraint1{0,
                                   {positions[0].x, positions[0].y, positions[0].z},
                                   {0.0, 0.0, 0.0}};
        constraints.push_back(constraint1);
    }

    void build_default_system(size_t points_num) {
        this->change_parameters(
                1.0, 0.05, 0.0, 500, 0.1, 2.0, 0.1
        );
        positions.resize(points_num);                                // each point corresponds to 1 pos, vel, f
        velocities.resize(points_num);
        forces.resize(points_num);
        edges.resize(_edge_num);                                      // but only corresponds to num-1 edges

        for (size_t i = 0; i != points_num; ++i) {
            positions[i].x = 0.5 + 0.5 * static_cast<double>(i) / (double)points_num; // evenly distribute points
            positions[i].y = 0.9;
        }
        for (size_t i = 0; i != _edge_num; ++i) {
            edges[i] = Edge{i, i + 1};           // connect points in a row
        }
        for (size_t i = 0; i != _edge_num; ++i) {
            rest_lengths.push_back((positions[edges[i].first_connect] - positions[edges[i].second_connect]).length());
        }
        PointConstraint constraint1{0,
                                    {positions[0].x, positions[0].y, positions[0].z},
                                    {0.0, 0.0, 0.0}};
        constraints.push_back(constraint1);
    }

    void export_state(std::vector<jet::Vector3D>& pos) {
        pos.resize(_points_num);                                    // resize to fit
        for (size_t i = 0; i != _points_num; ++i) {
            pos[i] = positions[i];
        }
    }

    size_t find_closet_point(const jet::Vector3D& pos) {
        size_t closet_index = 0;
        double closet_dis = (pos - positions[0]).length();
        for (size_t i = 1; i != _points_num; ++i) {
            double dis = (pos - positions[i]).length();
            if (dis < closet_dis) {
                closet_dis = dis;
                closet_index = i;
            }
        }
        return closet_index;
    }

    void change_wind(jet::VectorField3Ptr wind) {
        this->_wind = std::move(wind);
    }

    void change_parameters(double mass = 1.0, double rest_length = 1.0, double floor_pos = 0.0,
                           double stiffness = 1.0, double damping = 1.0, double air_friction = 1.0,
                           double restitution = 0.1) {
        _mass = mass;
        _rest_length = rest_length;
        _floor_pos = floor_pos;
        _stiffness = stiffness;
        _damping = damping;
        _air_friction = air_friction;
        _restitution = restitution;
    }

    void changeGravity(const jet::Vector3D& gravity) {
        this->_gravity = gravity;
    }

protected:
    void onAdvanceTimeStep(double timeIntervalInSeconds) override {
        using namespace jet;
        for (size_t i = 0; i != _points_num; ++i) {
            forces[i] = _mass * _gravity;                                   // first apply gravity

            Vector3D relative_vel = velocities[i];                          // then apply wind
            if (_wind != nullptr)
                relative_vel -= _wind->sample(positions[i]);

            forces[i] -= _air_friction * relative_vel;                     // air friction f = -b * v
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
                Vector3D force = -_stiffness * (distance - rest_lengths[i]) * r;
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
            Vector3D new_acc = forces[i] / _mass;                               // a = f / m
            Vector3D new_vel = velocities[i] + timeIntervalInSeconds * new_acc;
            Vector3D new_pos = positions[i] + timeIntervalInSeconds * new_vel;

            if (new_pos.y < _floor_pos) {                                       // prevent going through floor
                new_pos.y = _floor_pos;
                if (new_vel.y < 0.0) {                                          // velocity attenuation
                    new_vel *= -_restitution;
                    new_pos.y += timeIntervalInSeconds * new_vel.y;
                }
            }

            velocities[i] = new_vel;                                            // this is copy-by-value, no worry
            positions[i] = new_pos;
        }

        for (auto& constraint: constraints) {                   // apply constrains..
            size_t index = constraint.index;
            velocities[index] = constraint.fixed_vel;
            positions[index] = constraint.fixed_pos;
        }
    }

private:
    jet::Vector3D _gravity{0.0, -9.8, 0.0};
    jet::VectorField3Ptr _wind;
    double _mass;
    double _stiffness;
    double _damping;
    double _air_friction;
    double _rest_length;
    double _floor_pos;
    double _restitution;
    size_t _points_num;
    size_t _edge_num;
};

void test_simple_mass_spring_animation() {
    using namespace jet;
    using namespace visualime;
    std::mutex modification_mutex;
    SimpleMassSpringAnimation animation(3,
                                        std::make_shared<jet::ConstantVectorField3>(jet::Vector3D{0.0, 0.0, 0.0})
                                        );
    double infinity = std::numeric_limits<double>::infinity();
    animation.build_two_pendulum_system();
//    animation.build_default_system(3);
    jet::Frame frame;
    double fps_vis = 120;
    double frame_per_vis = 100;
    frame.timeIntervalInSeconds = 1 / (fps_vis * frame_per_vis);
//    SimpleMassSpringAnimation::PointConstraint constraint1{0,
//                   {animation.positions[0].x, animation.positions[0].y, animation.positions[0].z},
//                  {0.0, 0.0, 0.0}};
//    animation.constraints.push_back(constraint1);

    GLsizei RENDER_WIDTH = 1000;
    GLsizei RENDER_HEIGHT = 1000;

    scene::scene2d scene{RENDER_WIDTH, RENDER_HEIGHT, 1};

    std::vector<Vector3D> pos;
    animation.export_state(pos);
    std::vector<size_t> circle_index;

    std::vector<glm::vec<3, unsigned char>> colors {                                          // beautiful color of grey, red, green
            {176, 132, 169},
            {176, 132, 169},
            {176, 132, 169},
            {132, 176, 169}
    };

    for (int i = 0; i != pos.size(); ++i) {
        circle_index.push_back(scene.add_circle_normalized(
                                    colors[i], {pos[i].x, pos[i].y}, 0.2, 0.01));
    }
    std::vector<size_t> line_index;
    for (auto & edge : animation.edges) {
        line_index.push_back(scene.add_line_normalized(colors[3],
                               {pos[edge.first_connect].x, pos[edge.first_connect].y},
                               {pos[edge.second_connect].x, pos[edge.second_connect].y}, 0.1, 0.01));
    }

    std::function<void(const glm::vec2&)> add_point = [&](const glm::vec2& _pos_param) {
                                                                                // normalize and revert y
        glm::vec2 _pos{_pos_param.x / (float)RENDER_WIDTH, 1.0f - _pos_param.y / (float)RENDER_HEIGHT};
        size_t new_index = animation.positions.size();                          // connect to closet point
        size_t closet_index = animation.find_closet_point({_pos.x, _pos.y, 0.0});

        animation.positions.emplace_back(_pos.x, _pos.y, 0.0);       // emplace to end
        animation.velocities.emplace_back(0.0, 0.0, 0.0);
        animation.forces.emplace_back(0.0, 0.0, 0.0);                // connect!
        animation.edges.push_back(SimpleMassSpringAnimation::Edge{closet_index, new_index});
                                                                                // (silly)dont forget update _points_num..
        animation.update_number_of_points(animation.positions.size());

        circle_index.push_back(scene.add_circle_normalized({176, 132, 169},     // create circle
                                                           {_pos.x, _pos.y}, 0.2, 0.01));
        line_index.push_back(scene.add_line_normalized({132, 176, 169},         // create line
                                                       {animation.positions[closet_index].x,
                                                        animation.positions[closet_index].y},
                                                       {_pos.x, _pos.y}, 0.1, 0.01));
        animation.rest_lengths.push_back((animation.positions[closet_index] - animation.positions[new_index]).length());
    };

    std::function<void(const glm::vec2&)> on_mouse_left_click = [&](const glm::vec2& _pos_param) {
        modification_mutex.lock();
        add_point(_pos_param);
//        frame.advance();
        modification_mutex.unlock();
    };
    std::function<void(const glm::vec2&)> on_mouse_right_click = [&](const glm::vec2& _pos_param) {
        modification_mutex.lock();
        add_point(_pos_param);                                                  // add & constraint as stable
        size_t new_index = animation.positions.size() - 1;
        SimpleMassSpringAnimation::PointConstraint constraint{new_index, {animation.positions[new_index].x,
                                                                          animation.positions[new_index].y,
                                                                          animation.positions[new_index].z},
                                                              {0.0, 0.0, 0.0}};
        animation.constraints.push_back(constraint);
        modification_mutex.unlock();
    };
    scene.on_mouse_left_click = on_mouse_left_click;
    scene.on_mouse_right_click = on_mouse_right_click;
    scene.launch(true);

    const size_t TRACE_LENGTH = 5000;
    std::list<size_t> small_circles1;
    std::list<size_t> small_circles2;

    while (true) {
        if (!scene.is_running()) break;
        auto start_time = std::chrono::high_resolution_clock::now();
        modification_mutex.lock();
        for (int i = 0; i != static_cast<int>(frame_per_vis); ++i) {
            animation.update(frame);                                            // inner loop of animation updating
            frame.advance();
        }
        animation.export_state(pos);
        for (int j = 0; j != pos.size(); ++j) {
            scene.change_primitive_position_normalized(circle_index[j], {pos[j].x, pos[j].y});
        }
        for (int j = 0; j != animation.edges.size(); ++j) {
            scene.change_line_start_end(line_index[j],
                            {pos[animation.edges[j].first_connect].x, pos[animation.edges[j].first_connect].y},
                            {pos[animation.edges[j].second_connect].x, pos[animation.edges[j].second_connect].y});
        }
        bool res = scene.draw_primitive(std::make_shared<primitive::solid_circle>(
                glm::vec2{pos[2].x, pos[2].y},
                0.3, 0.003,
                glm::vec<3, unsigned char>{176, 132, 169}));
        if (!res) std::cout << "fatal: draw_primitive not implemented" << std::endl;
//        scene.add_circle_normalized(colors[1],
//                                    {pos[1].x, pos[1].y}, 0.1, 0.001);
//        scene.add_circle_normalized(colors[2],
//                                    {pos[2].x, pos[2].y}, 0.1, 0.001);
//        small_circles1.push_back(scene.add_circle_normalized(colors[1],
//                                     {pos[1].x, pos[1].y}, 0.1, 0.001));
//        small_circles2.push_back(scene.add_circle_normalized(colors[2],
//                                     {pos[2].x, pos[2].y}, 0.1, 0.001));

        // if small circles too many, delete the first one
//        if (small_circles1.size() > TRACE_LENGTH) {
//            scene.delete_primitive(small_circles1.front());
//            small_circles1.pop_front();
//        }
//        if (small_circles2.size() > TRACE_LENGTH) {
//            scene.delete_primitive(small_circles2.front());
//            small_circles2.pop_front();
//        }
        scene.refresh();
        modification_mutex.unlock();
        auto end_time = std::chrono::high_resolution_clock::now();
//        std::cout << "[SpingMass][performance] calc scene: "
//                << std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count()
//                << " us" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds (1));
//        std::this_thread::sleep_for(std::chrono::microseconds ((unsigned)(1000000 / (fps_vis))));
    }
    if (scene.get_run_thread().joinable()) {
        scene.get_run_thread().join();
    }
    std::cout << "Simple mass spring animation finished" << std::endl;
}


int main() {
    jet::Logging::mute();
//    test_sine_animation_visualime();
    test_simple_mass_spring_animation();
    return 0;
}