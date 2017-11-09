#include "world.h"
#include <iostream>
#include <algorithm>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <imgui.h>

#define R 8.314
float n = 32.0;
float jforce = 16.0;
float dratio = 1.0;
float bmass = 0.1;
float air_drag = 0.01;

sf::RenderWindow* debug_window;

Vector normalize(const Vector& source)
{
    float length = sqrt((source.x * source.x) + (source.y * source.y));
    if (length != 0)
        return Vector(source.x / length, source.y / length);
    else
        return source;
}

BodyVertex::BodyVertex(Vector position) {
    this->mass = bmass;
    this->damping_ratio = dratio;
    this->joint_force = jforce;
    this->position = position;
    this->velocity = Vector(0, 0);
}

void BodyVertex::set_joints(BodyVertex* body_a, BodyVertex* body_b) {
    this->joined_bodies[0] = body_a;
    this->joined_bodies[1] = body_b;

    this->original_distances[0] = (this->position - body_a->position).magnitude();
    this->original_distances[1] = (this->position - body_b->position).magnitude();
}

void BodyVertex::apply_force(float dt, Vector force) {
    this->velocity += force/mass*dt;
}

void draw_vector(Vector vec, Vector position) {
    auto size = debug_window->getSize();
    
    sf::RectangleShape line(sf::Vector2f(vec.magnitude()*5, 2));
    line.setPosition(position.x * 5 + size.x/2, -position.y * 5 + size.y/2);

    double angle;
    if (vec.x != 0) {
        angle = atan2(vec.y, vec.x);
    } else {
        angle = M_PI/2;
    }

    auto deg = angle * 180 / M_PI;
    line.rotate(-deg);
    
    line.setFillColor(sf::Color(255, 0, 0));

    debug_window->draw(line);
}

void BodyVertex::update(float dt, float volume) {
    this->mass = bmass;
    this->joint_force = jforce;
    this->damping_ratio = dratio;

    auto force = Vector(0, 0);
    float surface = 0.0;

    auto c = 2*this->damping_ratio*sqrt(this->mass*this->joint_force);
    for (int i = 0; i < 2; i++) {
        auto joined_body = this->joined_bodies[i];
        auto delta = this->position - joined_body->position;
        
        auto relative_velocity = this->velocity - joined_body->velocity;
        auto joint_velocity = delta.normalize() * relative_velocity.dot(delta.normalize());

        auto extention = delta.magnitude() - this->original_distances[i];
        
        // F = -kx - cv
        force += delta.normalize() * extention * -this->joint_force - joint_velocity*c;
        
        surface += delta.magnitude();
    }
    
    auto normal = -Vector(joined_bodies[0]->position.y - joined_bodies[1]->position.y,
                          joined_bodies[1]->position.x - joined_bodies[0]->position.x).normalize();
    
    auto intensity = R*n*surface/volume;
    force += normal*intensity;
    
    this->apply_force(dt, force);
    this->apply_force(dt, -velocity*air_drag);
    this->position += this->velocity * dt;

    if (this->position.y < 0) {
        this->position.y = 0;
        this->velocity.y = -this->velocity.y/2;
        this->velocity.x = this->velocity.x/2;
    }
}

Body::Body(unsigned int num_verts) {
    for (unsigned int i = 0; i < num_verts; i++) {
        float x = cos((float)i/(float)num_verts * 2 * M_PI) * 5;
        float y = sin((float)i/(float)num_verts * 2 * M_PI) * 5;
        auto position = Vector(x, y + 50);

        this->verts.push_back(
            BodyVertex(position)
        );
    }

    for (unsigned int i = 0; i < num_verts; i++) {
        auto prev_i = (int)i - 1;
        if (prev_i < 0) {
            prev_i = num_verts-1;
        }

        auto prev_body = &this->verts[prev_i];
        auto next_body = &this->verts[(i + 1) % num_verts];
        
        std::cout << prev_i << " " << (i+1) % num_verts << std::endl;
        this->verts[i].set_joints(prev_body, next_body);
    }
}

void Body::update(float dt) {
    ImGui::SliderFloat("Moles", &n, 0.1f, 128.0f);
    ImGui::SliderFloat("Damping ratio", &dratio, 0.0f, 128.0f);
    ImGui::SliderFloat("Joint force", &jforce, 0.0f, 128.0f);
    ImGui::SliderFloat("Mass", &bmass, 0.01f, 2.0f);
    ImGui::SliderFloat("Drag", &air_drag, 0.001f, 0.1f);
    
    auto volume = this->get_volume();
    
    for (unsigned int i = 0; i < this->verts.size(); i++) {
        this->verts[i].velocity.y -= 9.8*dt;
    }
    
    for (unsigned int i = 0; i < this->verts.size(); i++) {
        this->verts[i].update(dt, volume);
    }
}

float Body::get_volume() {
    float volume = 0.0;
    
    for (unsigned int i = 0; i < this->verts.size(); i++) {
        auto vertex_a = this->verts[i];
        auto vertex_b = this->verts[(i+1) % this->verts.size()];

        volume += vertex_a.position.x * vertex_b.position.y - vertex_a.position.y * vertex_b.position.x;
    }

    return abs(volume) / 2.0;
}