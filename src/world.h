#pragma once

#include <vector.h>
#include <stdint.h>
#include <optional>
#include <vector>
#include <array>

#include <SFML/Graphics.hpp>

extern sf::RenderWindow* debug_window;

class BodyVertex {
    public:
        float mass;
        Vector velocity;
        Vector position;

        float joint_force;
        float damping_ratio;
        std::array<double, 2> original_distances;
        std::array<BodyVertex*, 2> joined_bodies;
        
        BodyVertex(Vector position);
        void set_joints(BodyVertex* body_a, BodyVertex* body_b);
        void apply_force(float dt, Vector force);
        void update(float dt, float volume);
};

class Body {
    public:
        std::vector<BodyVertex> verts;

        Body(unsigned int num_verts);
        void update(float dt);
        float get_volume();
};