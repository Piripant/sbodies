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
        std::array<unsigned int, 2> joined_bodies;
        
        BodyVertex(Vector position);
        void apply_force(float dt, Vector force);
        void update(float dt);
};

class Body {
    public:
        std::vector<BodyVertex> verts;

        Body(unsigned int num_verts);
        void set_joints();
        void append_vertex(BodyVertex vertex);
        void update_vertex(BodyVertex& vertex, float dt, float volume);
        void update(float dt);
        float get_volume();
};