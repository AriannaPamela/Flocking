# Flocking with Artificial Potential Fields (APF)

This repository contains a simple implementation of a flocking simulation based on the double integrator model. In particular, in this simulation each agent follows two types of behaviors:

1. **Formation**: the agents want to arrange themselves according to a previously defined desired configuration.
2. **Speed consensus**: the agents want to all line up at a single group speed.

In this simulation there are seven agents that must arrange themselves in a hexagon with a seventh central node, respecting a set of 30 constraints useful for defining the desired rigid formation.The configuration is summarized in the following figure:

                   0..............1
                  . .           .  .
                 .    .       .     .
                .       .   .        .
              5. . . . . .6 . . . . . .2
                .       .   .        .
                 .     .       .    .
                  .  .           . .
                    4.............3


## Agent state representation

Each agent status is modelled through three components:

1. **Position**: its position in space represented by the coordinates x and y,
2. **Speed**: its speed also with speed_x and speed_y components,
3. **neighborhood**: vector of its neighbours which contains a subset of nodes to which the agent is directly connected (with these nodes has a distance constraint expressed by the rigid formation).

Positions and speed of agents are randomly generated at the beginning of the simulation, in particular the position is generated in a range that ensures that the agent is inside the screen, while the speeds are between 0 and 100.

## Behavior implementation

The two behaviors followed by the agents are modelled according to the artificial potentials method. More specifically, each agent is subject to two potentials:

1. **Formation**: It is used to get the desired formation to the agents, has minima in correspondence of the desired distance between a node and its neighbor. For each agent calculated as:  
$Σ_{j ∈ 𝒩ᵢ} \frac{1}{4}$  (||pᵢ - pⱼ||² − δᵢⱼ²)² (𝒩ᵢ neighborhood of agent i, pᵢ position of agent i, pⱼ position of agent j, δᵢⱼ desired distance between i and j)
	
3. **Speed**: Used to coordinate all agents at a common speed. In this case the potential used has a minimum in correspondence of the common speed and is of the type:   
$Σ_{j ∈ 𝒩ᵢ}$ ||vᵢ - vⱼ||²  (𝒩ᵢ neighborhood of agent i, vᵢ speed of agent i, vⱼ speed of agent j)
	
The potential of each agent is therefore the sum of the two potentials described above and its derivative turns out to be:
$-K_{for} * Σ_{j ∈ 𝒩ᵢ}$ (||pᵢ - pⱼ||² - δᵢⱼ²) * (pⱼ - pᵢ) - K_{vel} $Σ_{j ∈ 𝒩ᵢ}$ (vᵢ - vⱼ). The contribution of the two potentials is finally weighted by two constants (Kfor and Kvel). It is noted that the contribution of the formation component must be moderated in order not to incur oscillatory trends.

Finally, at the end of each iteration, the position and speed of each agent are updated as follows: 

p(t+1) = pᵢ(t) + T_s * vᵢ(t)

v(t+1) = vᵢ(t) + T_s * uᵢ(t)

where Ts is sampling time.

## Simulations

Some simulations follow. In order to be able to observe the agents even when they move away from the screen, the centre of the simulation screen has been fixed at the mean position of the agents. This means that it is not possible to see the agents moving at a consensus speed, as they appear stationary once the desired formation has been reached. In the last simulation, therefore, the center of the screen has been fixed and agents can be seen moving away in formation.

![Simulation](simulations/flocking(Kf=0.0001).gif)

Simulation with Kfor set to 0.0001 and Kvel=0.3


![Simulation](simulations/flocking(Kf=0.001).gif)

Simulation with Kfor set to 0.001 and Kvel=0.3. Oscillatory behaviour may be observed before reaching the desired formation.

![Simulation](simulations/flocking(kf=0.01).gif)

Simulation with Kfor set to 0.01 and Kvel=0.3. A more pronounced oscillatory behaviour may be observed.

![Simulation](simulations/flocking(3).gif)

Simulation with fixed camera center.





