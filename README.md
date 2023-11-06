# Reinforcement Learning for Ball Striking in a Golf-like Setting, Instructed by Assistant Professor Sarah Keren

This repository contains the code and resources for a research project focused on reinforcement learning (RL) in the context of ball striking, akin to golf. The project aims to teach a robot to strike a stationary ball with the optimal velocity and angle, allowing it to roll to a specified target point. The work is inspired by real-world robotics applications and utilizes a Gazebo-based simulation environment.

This project was conducted as part of our bachelor's degree program in Computer Science at the Technion - Israel Institute of Technology, in collaboration with the CLAIR laboratory (Collaborative Robotics & Artificial Intelligence Lab, [clair.cs.technion.ac.il](https://clair.cs.technion.ac.il/)). It was made possible with the invaluable guidance and support of Assistant Professor Sarah Keren. Her expertise and mentorship played a pivotal role in the successful execution of this research.

## Project Overview

- **Environment:** We've created a Gazebo simulation environment that closely mimics real-world conditions, including precise physics and ball attributes.

- **Reinforcement Learning:** The project employs RL techniques, including Deep Deterministic Policy Gradients (DDPG), Soft Actor-Critic (SAC), and Proximal Policy Optimization (PPO).

- **Iteration and Learning:** The system iteratively fine-tunes robot actions to maximize the chances of striking the ball with the right velocity and angle, leading it to the target point.

## System Design

The project involves a well-structured system design that governs its operation:
![System Design](https://github.com/assafBam/affordane-RL/assets/117855177/9e8fd853-822d-4e40-811f-0d2783b30fc8)
1. **Initialization:**
   - Begin by initializing the RL agent using the Stable Baselines 3 library.

2. **Exploration Phase:**
   - Generate random values for velocity (V) and degree (Theta) to serve as initial exploration parameters.

3. **Launching the Simulation:**
   - Initiates the execution of the run.py script, leading to the establishment of the Gazebo simulation environment meticulously tailored to mirror the physical attributes of our experiment's yellow ball. This precision extends to properties like the ball's radius, mass, coefficient of friction, moment of inertia, and more, all encapsulated within the AIR2.world file.

4. **Ball Striking Simulation (AIR2.py):**
   - Inside run.py, the AIR2.py script is launched. This script simulates the actions of the TurtleBot3 robot using the specified velocity (V) and degree (Theta) to strike the stationary ball.

5. **Monitoring the Simulation:**
   - Wait until the AIR2.py process is completed, indicating that the simulated ball striking action has been performed. This action is executed using the move_base and cmd_vel.

6. **Collecting Ball's Final Position:**
   - Retrieve the final position of the ball in the simulated environment.

7. **Reward Calculation:**
   - Provide the obtained ball position information to the RL agent.

8. **Reinforcement Learning:**
   - The RL agent uses the collected data to calculate a reward for the combination of velocity (V) and degree (Theta) used in the previous simulation run.

9. **Iteration for Convergence:**
   - Repeat the process by launching the simulation again, this time with a new set of velocity (V') and degree (Theta') that the RL agent provides based on the previous reward. This iterative process continues until the RL agent converges, indicating that it has learned the optimal velocity and degree for accurate ball striking.

10. **Stopping Criteria:**
    - The process stops when the RL agent has reached a predefined convergence criterion or a set number of epochs.

These sequential steps define the core functioning of our project, orchestrating the learning and improvement of the robot's ball-striking skills within the Gazebo simulation environment.

## Challenges Faced in the Project

The project presented several challenges that required innovative solutions:

1. **Real-World Modeling, Lack of Documentation, and Precise Moment of Inertia Adjustment:** Translating real-world lab attributes into the simulation was a major hurdle. The AIR2.world file needed to accurately depict the physical characteristics of our yellow ball, such as mass, radius, and moment of inertia. This process demanded a deep understanding of real-world attributes and Gazebo's world file format. The absence of comprehensive documentation for creating Gazebo world files added to the challenge. Meticulous tuning of the moment of inertia values was necessary to achieve realistic ball behavior.

2. **Process Management and Data Collection:** Coordinating multiple processes in parallel was complex, demanding careful process management to ensure they started, ran, and terminated as intended. Simultaneously, extracting the final values generated by the AIR2.py process was crucial for assessing simulation results. This required the establishment of effective data collection and storage mechanisms within the system.

3. **Navigating Unfamiliar Territory with Stable Baselines 3:** Working with Stable Baselines 3 presented a significant learning curve. The team encountered this reinforcement learning framework for the first time, requiring a steep learning curve. There was a lack of prior experience in developing code that integrated RL agents into a complex robotic system.

4. **Balancing Under-Fitting and Over-Fitting:** Achieving convergence in the RL agents was a multifaceted challenge. Finding the right balance between under-fitting and over-fitting was essential. Each of the three employed algorithms had unique training requirements and parameters, demanding a systematic exploration of epochs to ensure effective convergence. Rigorous experimentation was required to identify the optimal number of training iterations for each algorithm.

5. **Iterative Execution:** The system had to orchestrate multiple iterations of the Gazebo simulation, managing processes between each iteration while providing the necessary inputs.

6. **Robustness to Initial Conditions:** Ensuring that learned behaviors were robust to different initial conditions was a significant challenge. The RL agent needed to adapt to various configurations, including variations in the ball's target hole position.

These challenges demonstrate the depth and complexity of your research-oriented system, showcasing the diverse skills and expertise required to successfully complete the project.

## Repository Structure

- `/code`: This directory contains the source code for the project, including scripts for simulation and RL training.

- `/docs`: Here, you'll find the documentation and resources that support the project.

## Getting Started

To get started with our project, follow these steps:

1. Clone the repository to your local machine.
2. Refer to the `/code` directory for code and scripts.
3. Explore the `/docs` directory for detailed project documentation.

## License

This project is licensed under the [MIT License](LICENSE), allowing you to use the code and resources for your research and development.

## Contact

For inquiries or collaborations, feel free to contact us at [asafephraim@gmail.com] or [assaf.bamberger@gmail.com].

Happy experimenting!

Assaf Bamberger & Asaf Ephraim







####################################################################
####################################################################
####################################################################
####################################################################
####################################################################
####################################################################
####################################################################





# affordane-RL
Needed locations for each file:
1. Launch file - ~/my_ws/src/MRS_236609/launch/assignment1
2. World file - ~/my_ws/src/MRS_236609/worlds
3. Python file - ~/my_ws/src/MRS_236609/scripts

RL Problem Modeling and Experiment Protocol:
https://docs.google.com/document/d/1NxH732ylhoQes8UqfHj81YX21MqGW3dFeCUbb5oIsB8/edit?usp=sharing 
