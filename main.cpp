#include <iostream>
#include <random>
#include <vector>
#include <cmath>
#include <SFML/Graphics.hpp>

const int SCREEN_WIDTH = 640;
const int SCREEN_HEIGHT = 480;
const int NUM_AGENT = 7;

struct Agent {
    float x; // x coordinate of agent's position
    float y; // y coordinate of agent's position
    float speed_x; // agent's speed along x axe
    float speed_y; // agent's speed along y axe
    std::vector<Agent *> neighborhood; // vector of agents connected to this agent
};

Agent generate_agent() {
    Agent agent;
    agent.x = (float)(rand()) /((float)(RAND_MAX) / (SCREEN_WIDTH));
    agent.y = (float)(rand()) /((float)(RAND_MAX) / (SCREEN_HEIGHT));
    agent.speed_x = (float)(rand()) /((float)(RAND_MAX) / (100.0f));
    agent.speed_y = (float)(rand()) /((float)(RAND_MAX) / (100.0f));
    return agent;
}

void move(Agent &agent, float ts, std::vector<float> distances) {

    /*Compute agent acceleration*/
    /*
     * Jvel(v) = ∑ ||vi - vj||^2
     * Jfor(p) = ∑ 1/4[||pi -pj||^2 - δij^2]^2 (δij desired distance between i and j)
     * J =                 Kfor*Jfor(p(t))              +       Kvel*Jvel(v(t))
     * u(t) =        -Kfor*[δ(Jfor(p(t)]/δpi(t)         -  Kvel*[δ(Jvel(v(t)]/δvi(t)
     *      = -Kfor*[∑(||pi-pj||^2 - δij^2)*(pj-pi)]    -       Kvel*[2*∑vi-vj]
     */

    float K_for = 0.0001;
    float K_vel = 0.30;
    float max_speed = 100.0;

    float formation_acc_x = 0; // formation component of acceleration along x axe -->  -Kfor*[∑(||pi-pj||^2 - δij^2)*(pj-pi)]
    float formation_acc_y = 0; // formation component of acceleration along  y axe
    float vel_acc_x = 0; // velocity component of acceleration along x axe -->  Kvel*[2*∑vi-vj]
    float vel_acc_y = 0;  // velocity component of acceleration along y axe

    for (int i = 0; i < agent.neighborhood.size(); i++) {
        float dist_x = agent.neighborhood[i]->x - agent.x;
        float dist_y = agent.neighborhood[i]->y - agent.y;
        float distance = std::sqrt(pow(dist_x,2) + pow(dist_y, 2)); //Actual distance between i and j
        float desired_distance = distances[i];

        /*Compute formation potential*/
        float delta = (pow(distance,2) - pow(desired_distance,2)); //actual misalignment between squared desired distance (δij) and squared real distance
        formation_acc_x += delta * (-dist_x);
        formation_acc_y += delta * (-dist_y);

        /*Compute velocity potential to align agent's velocity -->  Kvel*[2*∑vi-vj] */
        vel_acc_x += - (agent.neighborhood[i]->speed_x - agent.speed_x);
        vel_acc_y += - (agent.neighborhood[i]->speed_y - agent.speed_y);

    }
    formation_acc_x *= -K_for;
    formation_acc_y *= -K_for;
    vel_acc_x *= -K_vel;
    vel_acc_y *= -K_vel;

    /*Total acceleration*/
    float acc_x = (formation_acc_x + vel_acc_x);
    float acc_y = (formation_acc_y + vel_acc_y);

    /*Update agent's position: p(t+1) = pi(t)+Ts*vi(t)*/
    agent.x = agent.x + ts * agent.speed_x ;
    agent.y = agent.y + ts * agent.speed_y ;

    /*Update agent's speed: v(t+1) = vi(t) + Ts * ui(t)*/
    agent.speed_x = agent.speed_x + ts * acc_x;
    agent.speed_y = agent.speed_y + ts * acc_y;

    /*Limit speed to a maximum speed*/
    float speed = std::sqrt(agent.speed_x * agent.speed_x + agent.speed_y * agent.speed_y);
    if (speed > max_speed) {
        agent.speed_x = (agent.speed_x / speed) * max_speed;
        agent.speed_y = (agent.speed_y / speed) * max_speed;
    }

}

int main() {

    std::random_device os_seed;
    const uint32_t seed = os_seed();
    srand(seed);

    /*Generate agents*/
    Agent agents[NUM_AGENT];
    for (int i = 0; i < NUM_AGENT; i++) {
        agents[i] = generate_agent();
    }

    /*Define a rigid formation
     *
     *             0..............1
     *            . .           .  .
     *           .    .       .     .
     *          .       .   .        .
     *        5. . . . . .6 . . . . . .2
     *          .       .   .        .
     *           .     .       .    .
     *            .  .           . .
     *              4.............3
     *
     *
     */

    /*Define agent's neighborhood*/
    agents[0].neighborhood = {&agents[1], &agents[3], &agents[5], &agents[6]};
    agents[1].neighborhood = {&agents[0], &agents[2], &agents[4], &agents[6]};
    agents[2].neighborhood = {&agents[1], &agents[3], &agents[5], &agents[6]};
    agents[3].neighborhood = {&agents[0], &agents[2], &agents[4], &agents[6]};
    agents[4].neighborhood = {&agents[1], &agents[3], &agents[5], &agents[6]};
    agents[5].neighborhood = {&agents[0], &agents[2], &agents[4], &agents[6]};
    agents[6].neighborhood = {&agents[0], &agents[1], &agents[2], &agents[3], &agents[4], &agents[5]};

    std::vector<float> distances[7];
    distances[0].push_back(80);
    distances[0].push_back(160);
    distances[0].push_back(80);
    distances[0].push_back(80);
    distances[1].push_back(80);
    distances[1].push_back(80);
    distances[1].push_back(160);
    distances[1].push_back(80);
    distances[2].push_back(80);
    distances[2].push_back(80);
    distances[2].push_back(160);
    distances[2].push_back(80);
    distances[3].push_back(160);
    distances[3].push_back(80);
    distances[3].push_back(80);
    distances[3].push_back(80);
    distances[4].push_back(160);
    distances[4].push_back(80);
    distances[4].push_back(80);
    distances[4].push_back(80);
    distances[5].push_back(80);
    distances[5].push_back(160);
    distances[5].push_back(80);
    distances[5].push_back(80);
    distances[6].push_back(80);
    distances[6].push_back(80);
    distances[6].push_back(80);
    distances[6].push_back(80);
    distances[6].push_back(80);
    distances[6].push_back(80);


    sf::RenderWindow window(sf::VideoMode(SCREEN_WIDTH, SCREEN_HEIGHT), "Agent Movement");
    window.setFramerateLimit(60);
    sf::CircleShape shape(5.0f);
    shape.setFillColor(sf::Color::Green);

    sf::View view(sf::FloatRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT));
    window.setView(view);


    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        /*Move the agents*/
        int id_agent = 0;
        for (auto &agent : agents) {
            move(agent, 1.0f / 60, distances[id_agent]);
            id_agent++;
        }

        // Agents' midpoint
        float center_x = 0.0f;
        float center_y = 0.0f;
        for (const auto &agent : agents) {
            center_x += agent.x;
            center_y += agent.y;
        }
        center_x /= NUM_AGENT;
        center_y /= NUM_AGENT;

        // Focus the camera on the agents
        view.setCenter(center_x, center_y);
        window.setView(view);

        window.clear();
        /*Print the agents*/
        for (const auto &agent : agents) {
            shape.setPosition(agent.x - shape.getRadius(), agent.y - shape.getRadius());
            window.draw(shape);
        }
        window.display();
    }


}
