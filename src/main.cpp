#include "world.h"
#include <SFML/Graphics.hpp>
#include <iostream>
#include <imgui.h>
#include <imgui-SFML.h>

int scale = 20;
sf::Color colors[2] = {
    sf::Color(0, 0, 0),
    sf::Color(0, 0, 255)
};

float step_time = 0.1;
bool mouse_pressed = false;
int press_y = 0;
int press_x = 0;

int nvertex = 3;
float sim_speed = 1.0;

bool edit_mode = true;

int main() {
	// create the window
    sf::RenderWindow window(sf::VideoMode(800, 600), "tfluids");
    debug_window = &window;
    ImGui::SFML::Init(window);

    auto body = Body(nvertex);
    std::cout << body.get_volume() << std::endl;

    int steps = 0;
    float elapsed = 0;
    sf::Clock delta_clock;
    // run the program as long as the window is open
    while (window.isOpen()) {
        auto elapsed = delta_clock.restart();
        auto dt = elapsed.asSeconds();
        if (dt > 0.001) {
            dt = 0;
        }

        dt *= sim_speed;
        
        auto size = window.getSize();
        sf::Event event;
        while (window.pollEvent(event)) {
            ImGui::SFML::ProcessEvent(event);

            if (event.type == sf::Event::Closed) {
                window.close();
            }

            if (event.type == sf::Event::Resized) {
              window.setView(sf::View(sf::FloatRect(0, 0, event.size.width, event.size.height)));
            }
            
            if (event.type == sf::Event::MouseButtonReleased && edit_mode) {
                /*
                if (event.mouseButton.button == sf::Mouse::Button::Left) {
                    auto x = (double)((double)event.mouseButton.x - size.x/2.0) / 5.0;
                    auto y = (double)((double)-event.mouseButton.y + size.y/2.0) / 5.0;

                    body.append_vertex(BodyVertex(Vector(x, y)));
                }
                */
                if (event.mouseButton.button == sf::Mouse::Button::Right) {
                    body.set_joints();
                    edit_mode = false;
                }
                
            }
        }

        ImGui::SFML::Update(window, elapsed);

        ImGui::Begin("Settings");
        if (ImGui::Button("Restart")) {
            body = Body(nvertex);
            edit_mode = true;
        }

        ImGui::Text("FPS: %f", 1/dt);
        ImGui::SliderFloat("Simulation speed", &sim_speed, 0.0f, 16.0f);
        ImGui::SliderInt("Number of vertexes", &nvertex, 1, 64);

        window.clear(sf::Color::White);
        if (!edit_mode) {
            body.update(dt);
        }

        ImGui::End();
        

        sf::ConvexShape body_shape;
        body_shape.setFillColor(sf::Color(200, 200, 200));
        body_shape.setPointCount(body.verts.size());
        for (int i = 0; i < body.verts.size(); i++) {
            auto x = body.verts[i].position.x * 5 + size.x / 2;
            auto y = -body.verts[i].position.y * 5 + size.y / 2;
            
            sf::RectangleShape point(sf::Vector2f(5, 5));
            point.setFillColor(sf::Color::Red);
            point.setPosition(sf::Vector2f(x - 5.0/2, y - 5.0/2));

            window.draw(point);

            body_shape.setPoint(i, sf::Vector2f(x, y));
        }
        window.draw(body_shape);

        ImGui::SFML::Render(window);
        window.display();
    }

	return 0;
}