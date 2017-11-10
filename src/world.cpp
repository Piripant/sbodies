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

void BodyVertex::apply_force(float dt, Vector force) {
    this->velocity += force/mass*dt;
}

void BodyVertex::update(float dt) {
    this->position += this->velocity * dt;
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

void Body::update_vertex(BodyVertex& vertex, float dt, float volume) {
    vertex.mass = bmass;
    vertex.joint_force = jforce;
    vertex.damping_ratio = dratio;

    auto force = Vector(0, 0);
    float surface = 0.0;

    auto c = 2*vertex.damping_ratio*sqrt(vertex.mass*vertex.joint_force);
    for (int i = 0; i < 2; i++) {
        auto joined_body = verts[vertex.joined_bodies[i]];
        auto delta = vertex.position - joined_body.position;
        
        auto relative_velocity = vertex.velocity - joined_body.velocity;
        auto joint_velocity = delta.normalize() * relative_velocity.dot(delta.normalize());

        auto extention = delta.magnitude() - vertex.original_distances[i];
        
        // F = -kx - cv
        force += delta.normalize() * extention * -vertex.joint_force - joint_velocity*c;
        
        surface += delta.magnitude();
    }
    
    auto normal = -Vector(verts[vertex.joined_bodies[0]].position.y - verts[vertex.joined_bodies[1]].position.y,
                          verts[vertex.joined_bodies[1]].position.x - verts[vertex.joined_bodies[0]].position.x).normalize();
    

    draw_vector(normal*5, vertex.position);
    auto intensity = R*n*surface/volume;
    force += normal*intensity;
    
    vertex.apply_force(dt, force);
    vertex.apply_force(dt, -vertex.velocity*air_drag);
    vertex.update(dt);

    if (vertex.position.y < 0) {
        vertex.position.y = 0;
        vertex.velocity.y = -vertex.velocity.y/2;
        vertex.velocity.x = vertex.velocity.x/2;
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
    
}

void Body::append_vertex(BodyVertex vertex) {
    this->verts.push_back(vertex);
}

void Body::set_joints() {
    for (unsigned int i = 0; i < verts.size(); i++) {
        auto next_i = (i + 1) % verts.size();
        auto prev_i = (int)i - 1;
        if (prev_i < 0) {
            prev_i = verts.size()-1;
        }

        auto prev_body = &this->verts[prev_i];
        auto next_body = &this->verts[next_i];
        
        verts[i].joined_bodies[0] = prev_i;
        verts[i].joined_bodies[1] = next_i;

        verts[i].original_distances[0] = (verts[i].position - verts[next_i].position).magnitude();
        verts[i].original_distances[1] = (verts[i].position - verts[prev_i].position).magnitude();

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
        this->update_vertex(verts[i], dt, volume);
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