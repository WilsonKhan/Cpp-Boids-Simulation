#include <iostream> 
#include <cstdlib>  
#include <Eigen/Dense>
#include <SFML/Graphics.hpp>
#include <vector>
#include <math.h>  

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std;
using namespace Eigen;

class Boid {

public:

    Boid(float initial_x, float initial_y, float initial_vx, float initial_vy) {

        // Initialise coordinates and movement vectors along with interaction radii
        coords = { initial_x, initial_y };
        velocity = { initial_vx, initial_vy };
        acceleration = { 0, 0 };
        radius = 7;
        attraction_radius = 100;
        repulsion_radius = 60;
        critical_repulsion_radius = 20;
    }

    // Calculates a polar vector between boids (magnitude and angle)
    pair<float, float> calculateVector(const Boid& boid, const Boid& otherBoid) {
        float dx = otherBoid.coords[0] - boid.coords[0];
        float dy = otherBoid.coords[1] - boid.coords[1];

        float magnitude = sqrt(dx * dx + dy * dy);
        float theta = atan2(dy, dx);

        return std::make_pair(magnitude, theta);
    }

    // Boid neighbour allignment for flock-like behaviour
    void align(const vector<Boid>& otherBoids) {
        float alignment_radius = 80;

        // Calculate average velocity of neighbors
        Vector2f avgVelocity(0, 0);
        uint8_t count = 0;

        for (const Boid& otherBoid : otherBoids) {
            if (&otherBoid != this) {
                pair<float, float> polar_vector = calculateVector(*this, otherBoid);
                float distance = polar_vector.first;

                if (distance <= alignment_radius) {
                    avgVelocity[0] += otherBoid.velocity[0];
                    avgVelocity[1] += otherBoid.velocity[1];
                    count++;
                }
            }
        }

        // Adjust the boid's velocity based on average velocity
        if (count > 0) {
            avgVelocity /= count;

            float alignment_factor = 2.5;
            acceleration[0] += alignment_factor * (avgVelocity[0] - velocity[0]);
            acceleration[1] += alignment_factor * (avgVelocity[1] - velocity[1]);
        }
    }

    void update(float deltaTime, float screenWidth, float screenHeight, const vector<Boid>& otherBoids) {

        //Sets parameters(Move into class attributes)
        float max_speed = 100;
        float attraction_damping_factor = 10;
        float repulsion_damping_factor = 15;
        float critical_repulsion_damping_factor = 2;

        // For each neighbour boid, calculate the polar vector to it
        for (const Boid& otherBoid : otherBoids) {
            if (&otherBoid != this) {
                pair<float, float> polar_vector = calculateVector(*this, otherBoid);
                float distance = polar_vector.first;
                float theta = polar_vector.second;

                // Attraction
                if (distance > repulsion_radius && distance <= attraction_radius) {
                    float magnitude = attraction_damping_factor * (attraction_radius - distance) / attraction_radius;
                    acceleration[0] += cos(theta) * magnitude;
                    acceleration[1] += sin(theta) * magnitude;
                }

                // Repulsion
                else if (distance <= repulsion_radius) { 

                    float magnitude = repulsion_damping_factor * (repulsion_radius - distance) / repulsion_radius;
                    acceleration[0] -= cos(theta) * magnitude;
                    acceleration[1] -= sin(theta) * magnitude;
                }

                // Critical Repulsion
                else if (distance <= critical_repulsion_radius) {
                    // Exponential repulstion factor (gets very strong as they boids get close when compared to linear growth)
                    float magnitude = critical_repulsion_damping_factor * exp(-distance / repulsion_radius);
                }
            }
        }

        align(otherBoids);

        // Update position and velocity
        velocity[0] += acceleration[0] * deltaTime;
        velocity[1] += acceleration[1] * deltaTime;

        // Normalize the velocity
        float speed = sqrt(velocity[0] * velocity[0] + velocity[1] * velocity[1]);
        if (speed > max_speed) {
            velocity[0] = (velocity[0] / speed) * max_speed;
            velocity[1] = (velocity[1] / speed) * max_speed;
        }

        coords[0] += velocity[0] * deltaTime;
        coords[1] += velocity[1] * deltaTime;

        // Screen wrap-around

        if (coords[0] > screenWidth) {
            coords[0] = 0;
        }

        if (coords[0] < 0) {
            coords[0] = screenWidth;
        }

        if (coords[1] > screenHeight) {
            coords[1] = 0;
        }

        if (coords[1] < 0) {
            coords[1] = screenHeight;
        }
    }

    void render(sf::RenderWindow& window, float screenWidth, float screenHeight) {
        // Create a convex shape to represent the boid arrow
        sf::ConvexShape arrow(4);

        // Calculate hue based on boid's position
        float hue = fmod(atan2(coords[1] - screenHeight / 2, coords[0] - screenWidth / 2) + M_PI, 2 * M_PI) / (2 * M_PI);

        // Convert HSV to RGB
        float r, g, b;
        float c = 1.0f;
        float x = (1.0f - fabs(fmod(6.0f * hue, 2.0f) - 1.0f));
        float m = 0.0f;

        if (hue < 1.0f / 6.0f) {
            r = c; g = x; b = 0.0f;
        }
        else if (hue < 2.0f / 6.0f) {
            r = x; g = c; b = 0.0f;
        }
        else if (hue < 3.0f / 6.0f) {
            r = 0.0f; g = c; b = x;
        }
        else if (hue < 4.0f / 6.0f) {
            r = 0.0f; g = x; b = c;
        }
        else if (hue < 5.0f / 6.0f) {
            r = x; g = 0.0f; b = c;
        }
        else {
            r = c; g = 0.0f; b = x;
        }

        // Adjust brightness
        r = (r + m) * 255;
        g = (g + m) * 255;
        b = (b + m) * 255;

        arrow.setFillColor(sf::Color(static_cast<uint8_t>(r), static_cast<uint8_t>(g), static_cast<uint8_t>(b)));

        arrow.setPosition(coords[0], coords[1]);

        // Set the vertices to form a little arrow
        float arrowSize = 7.0f;
        arrow.setPoint(0, sf::Vector2f(-arrowSize / 2.0f, -radius));       // Tip of the arrow
        arrow.setPoint(1, sf::Vector2f(arrowSize / 2.0f, -radius));        // Tip of the arrow
        arrow.setPoint(2, sf::Vector2f(0, radius));                        // Bottom
        arrow.setPoint(3, sf::Vector2f(-arrowSize / 2.0f, -radius));       // Back to the tip

        // Rotate the arrow according to boid's velocity
        float angle = atan2(velocity[1], velocity[0]) * (180.0 / M_PI);
        arrow.setRotation(angle - 90); // Change angle because of the order in which SFML render vertices
        
        window.draw(arrow);
    }


private:

    // Boid characteristics
    vector<float> coords{ 2 };
    vector<float> velocity{ 2 };
    vector<float> acceleration{ 2 };
    uint8_t radius;
    uint32_t attraction_radius;
    uint32_t repulsion_radius;
    uint32_t critical_repulsion_radius;
};

// Initialises boids in a grid with slight random deviation in position and velocity
void initialiseBoids(int boids_x, int boids_y, float screenWidth, float screenHeight, std::vector<Boid>& boids) {
    for (uint8_t i = 0; i < boids_x; i++) {
        for (uint8_t j = 0; j < boids_y; j++) {
            Boid boid(i * (screenWidth / boids_x) + rand() % 100, j * (screenHeight / boids_y) + rand() % 100, rand() % 100, rand() % 100);
            boids.push_back(boid);
        }
    }
}

// Initialise boids into the boids vector
vector<Boid> boids;

// Sets the initial amount of boids by defining the size of the array of boids that will be generated
uint16_t boids_x = 20;
uint16_t boids_y = 20;

int main() {

    // Get the desktop mode
    sf::VideoMode desktopMode = sf::VideoMode::getDesktopMode();

    // Retrieve the screen width and height
    uint16_t screenWidth = desktopMode.width;
    uint16_t screenHeight = desktopMode.height;

    sf::RenderWindow window(sf::VideoMode(screenWidth, screenHeight), "Boids Simulation", sf::Style::Fullscreen);

    initialiseBoids(boids_x, boids_y, screenWidth, screenHeight, boids);

    sf::Clock clock;

    while (window.isOpen()) {

        // Link the frame-rate and simulation increments
        sf::Time elapsed = clock.restart();
        float deltaTime = elapsed.asSeconds();

        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        window.clear(sf::Color::Black);

        std::random_shuffle(boids.begin(), boids.end());

        // Update and render the boids
        for (Boid& boid : boids) {
            boid.render(window, screenWidth, screenHeight);
            boid.update(deltaTime, screenWidth, screenHeight, boids);
        }

        window.display();
    }

    return 0;
}

