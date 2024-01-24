#include "jet/jet.h"
#include "glm/glm.hpp"
#include "iostream"
#include "visualime/visualime.h"
#include "functional"
#include "mutex"

glm::vec<3, unsigned char> mapVelocityToColor(const jet::Vector3D& vel, double normalize_factor = 5)
{
    double amplitude = vel.length() / normalize_factor;
    if (amplitude > 1.0) amplitude = 1.0;
    // faster, more red, slower, more blue
    return glm::vec<3, unsigned char>{255 * amplitude, 0, 0}
         + glm::vec<3, unsigned char>{0, 0, 255 * (1.0 - amplitude)};
}

glm::vec<3, unsigned char> mapVelocityToColor(const jet::Vector2D& vel, double normalize_factor = 5)
{
    double amplitude = vel.length() / normalize_factor;
    if (amplitude > 1.0) amplitude = 1.0;
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

void NaiveSPHSystem3() {
    using namespace jet;
    using namespace visualime;
    const double targetSpacing = 0.015;
    BoundingBox2D domain(Vector2D(), Vector2D(1, 1));

    std::mutex modification_mutex;
    PciSphSolver2 solver;
    solver.setPseudoViscosityCoefficient(0.0);

    SphSystemData2Ptr particles = solver.sphSystemData();
    particles->setTargetDensity(1000.0);
    particles->setTargetSpacing(targetSpacing);

    // Initialize source2
    ImplicitSurfaceSet2Ptr surfaceSet = std::make_shared<ImplicitSurfaceSet2>();
    surfaceSet->addExplicitSurface(
            std::make_shared<Plane2>(
                    Vector2D(0, 1), Vector2D(0, 0.25 * domain.height())));
    surfaceSet->addExplicitSurface(
            std::make_shared<Sphere2>(
                    domain.midPoint() + Vector2D{0, 0.35}, 0.15 * domain.width()));

    BoundingBox2D sourceBound(domain);
    sourceBound.expand(-targetSpacing);

    auto emitter = std::make_shared<VolumeParticleEmitter2>(
            surfaceSet,
            sourceBound,
            targetSpacing,
            Vector2D());
    solver.setEmitter(emitter);

    // Initialize boundary
    Box2Ptr box = std::make_shared<Box2>(domain);
    box->isNormalFlipped = true;
    RigidBodyCollider2Ptr collider = std::make_shared<RigidBodyCollider2>(box);
    solver.setCollider(collider);

    // Make it fast, but stable
//    solver.setViscosityCoefficient(0.3);
    solver.setTimeStepLimitScale(25.0);

    Frame frame;
    double timeIntervalInSeconds = 1.0 / 60.0;
    long long timeIntervalInMilliSeconds = 16;
    frame.timeIntervalInSeconds = timeIntervalInSeconds;

//    particles->addParticle({0.5, 1, 0}, Vector3D{}, Vector3D{});
    GLsizei width = 1200, height = 1200;
    scene::scene2d scene{width, height, 1.0, false};
    std::vector<unsigned> circle_index;
    std::function<void(const glm::vec2&, double)> add_circle = [&] (const glm::vec2& pos, double r) -> void {
        modification_mutex.lock();
        particles->addParticle(
                {pos.x / width, 1.0f - pos.y / height},
                Vector2D{}, Vector2D{}
        );
        circle_index.emplace_back(
                scene.add_circle_normalized(
                        {176, 132, 169},
                        glm::vec2{pos.x / width, 1.0f - pos.y / height},
                        0.1, r)
        );
        modification_mutex.unlock();
    };
    std::function<void(const glm::vec2&)> left_click = [&](const glm::vec2& pos) { add_circle(pos, 0.005); };
    std::function<void(const glm::vec2&)> right_click = [&](const glm::vec2& pos) { add_circle(pos, 0.03); };
    scene.on_mouse_left_click = left_click;
    scene.on_mouse_right_click = right_click;
    for(auto& pos: particles->positions())
    {
        circle_index.emplace_back(
                scene.add_circle_normalized({176, 132, 169}, glm::vec2{pos.x, pos.y}, 0.1, 0.01)
        );
    }
    scene.launch(true);

    if (!scene.is_running())
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    int counter = 0;
    while (true)
    {
        modification_mutex.lock();
        auto start = std::chrono::system_clock::now();
        solver.update(frame);
        auto end = std::chrono::system_clock::now();
        std::cout << "[info] solver.update(frame) takes "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
                  << " milliseconds" << std::endl;
        // first add the emitted particles in circle_index
        for (size_t i = circle_index.size(); i < particles->positions().size(); ++i) {
            auto pos = particles->positions()[i];
            circle_index.emplace_back(
                    scene.add_circle_normalized(
                            {176, 132, 169},
                            glm::vec2{pos.x / width, 1.0f - pos.y / height},
                            0.1, 0.005)
            );
        }
        std::cout << "force: " << particles->forces()[0].x << ", " << particles->forces()[0].y << std::endl;
        for (size_t i = 0; i != circle_index.size(); ++i)
        {
            scene.change_primitive_position_normalized(
                    circle_index[i],
                    {particles->positions()[i].x, particles->positions()[i].y}
            );
            scene.change_primitive_color(circle_index[i],
                                         mapVelocityToColor(particles->velocities()[i], 1));
        }
        scene.refresh();
        frame.advance();
        modification_mutex.unlock();

//        std::this_thread::sleep_for(std::chrono::milliseconds(timeIntervalInMilliSeconds));
//        std::cin >> counter;
        if (!scene.is_running()) {
            break;
        }
    }
    std::cout << "NaiveSPHSystem3, naived" << std::endl;
}


void RotatingTankPCISPH() {
    using namespace jet;
    using namespace visualime;
    const double targetSpacing = 0.01;
    std::mutex modification_mutex;
    // Build solver
    auto solver = PciSphSolver2::builder()
            .withTargetSpacing(targetSpacing)
            .makeShared();
    SphSystemData2Ptr particles = solver->sphSystemData();
    solver->setViscosityCoefficient(0.01);
    solver->setTimeStepLimitScale(30.0);

    Frame frame;
    double timeIntervalInSeconds = 1.0 / 60.0;
    long long timeIntervalInMilliSeconds = 16;
    frame.timeIntervalInSeconds = timeIntervalInSeconds;

    // Build emitter
    auto box = Box2::builder()
            .withLowerCorner({0.25 + targetSpacing, 0.25 + targetSpacing})
            .withUpperCorner({0.75 - targetSpacing, 0.50})
            .makeShared();

    auto emitter = VolumeParticleEmitter2::builder()
            .withSurface(box)
            .withSpacing(targetSpacing)
            .withIsOneShot(true)
            .makeShared();

    solver->setEmitter(emitter);

    // Build collider
    auto tank = Box2::builder()
            .withLowerCorner({-0.25, -0.25})
            .withUpperCorner({ 0.25,  0.25})
            .withTranslation({0.5, 0.5})
            .withOrientation(0.0)
            .withIsNormalFlipped(true)
            .makeShared();

    auto collider = RigidBodyCollider2::builder()
            .withSurface(tank)
            .withAngularVelocity(2.0)
            .makeShared();

    collider->setOnBeginUpdateCallback([] (Collider2* col, double t, double) {
        col->surface()->transform.setOrientation(4.0 * t);
        dynamic_cast<RigidBodyCollider2*>(col)->angularVelocity = 4.0;
    });

    solver->setCollider(collider);

    GLsizei width = 1200, height = 1200;
    scene::scene2d scene{width, height, 1.0, false};
    std::vector<unsigned> circle_index;
    std::function<void(const glm::vec2&, double)> add_circle = [&] (const glm::vec2& pos, double r) -> void {
        modification_mutex.lock();
        particles->addParticle(
                {pos.x / width, 1.0f - pos.y / height},
                Vector2D{}, Vector2D{}
        );
        circle_index.emplace_back(
                scene.add_circle_normalized(
                        {176, 132, 169},
                        glm::vec2{pos.x / width, 1.0f - pos.y / height},
                        0.1, r)
        );
        modification_mutex.unlock();
    };
    std::function<void(const glm::vec2&)> left_click = [&](const glm::vec2& pos) { add_circle(pos, 0.005); };
    std::function<void(const glm::vec2&)> right_click = [&](const glm::vec2& pos) { add_circle(pos, 0.03); };
    scene.on_mouse_left_click = left_click;
    scene.on_mouse_right_click = right_click;
    for(auto& pos: particles->positions())
    {
        circle_index.emplace_back(
                scene.add_circle_normalized({176, 132, 169}, glm::vec2{pos.x, pos.y}, 0.1, 0.01)
        );
    }
    scene.launch(true);

    if (!scene.is_running())
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    while (true)
    {
        modification_mutex.lock();
        auto start = std::chrono::system_clock::now();
        solver->update(frame);
        auto end = std::chrono::system_clock::now();
        std::cout << "[info] solver.update(frame) takes "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
                  << " milliseconds" << std::endl;
        // first add the emitted particles in circle_index
        for (size_t i = circle_index.size(); i < particles->positions().size(); ++i) {
            auto pos = particles->positions()[i];
            circle_index.emplace_back(
                    scene.add_circle_normalized(
                            {176, 132, 169},
                            glm::vec2{pos.x / width, 1.0f - pos.y / height},
                            0.1, 0.005)
            );
        }
        std::cout << "force: " << particles->forces()[0].x << ", " << particles->forces()[0].y << std::endl;
        for (size_t i = 0; i != circle_index.size(); ++i)
        {
            scene.change_primitive_position_normalized(
                    circle_index[i],
                    {particles->positions()[i].x, particles->positions()[i].y}
            );
            scene.change_primitive_color(circle_index[i],
                                         mapVelocityToColor(particles->velocities()[i], 1));
        }
        scene.refresh();
        frame.advance();
        modification_mutex.unlock();

        if (!scene.is_running()) {
            break;
        }
    }
}


int main() {
    jet::Logging::mute();
//    FirstSightOnParticleSystemSolver3();
//    NaiveSPHSystem3();
    RotatingTankPCISPH();
    return 0;
}


