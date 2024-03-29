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

1. **Real-World Modeling, Lack of Documentation, and Precise Moment of Inertia Adjustment:** Translating real-world lab attributes into the simulation was a significant hurdle. The AIR2.world file needed to accurately depict the physical characteristics of our yellow ball, such as mass, radius, and moment of inertia. This process demanded a deep understanding of real-world attributes and Gazebo's world file format. The absence of comprehensive documentation for creating Gazebo world files added to the challenge. Meticulous tuning of the moment of inertia values was necessary to achieve realistic ball behavior.

2. **Process Management and Data Collection:** Coordinating multiple processes in parallel was complex, demanding careful process management to ensure they started, ran, and terminated as intended. Simultaneously, extracting the final values generated by the AIR2.py process was crucial for assessing simulation results. This required the establishment of effective data collection and storage mechanisms within the system.

3. **Navigating Unfamiliar Territory with Stable Baselines 3:** Working with Stable Baselines 3 presented a significant learning curve. The team encountered this reinforcement learning framework for the first time, requiring a steep learning curve. There was a lack of prior experience in developing code that integrated RL agents into a complex robotic system.

4. **Balancing Under-Fitting and Over-Fitting:** Achieving convergence in the RL agents was a multifaceted challenge. Finding the right balance between under-fitting and over-fitting was essential. Each of the three employed algorithms had unique training requirements and parameters, demanding a systematic exploration of epochs to ensure effective convergence. Rigorous experimentation was required to identify the optimal number of training iterations for each algorithm.

5. **Iterative Execution:** The system had to orchestrate multiple iterations of the Gazebo simulation, managing processes between each iteration while providing the necessary inputs.

6. **Robustness to Initial Conditions:** Ensuring that learned behaviors were robust to different initial conditions was a significant challenge. The RL agent needed to adapt to various configurations, including variations in the ball's target hole position.

These challenges demonstrate the depth and complexity of your research-oriented system, showcasing the diverse skills and expertise required to complete the project.

## Repository Structure
Explore the organization of our project repository:

- `/docs`
This directory houses documentation and additional project resources to support your understanding of the work.

- `/code`
In this directory, you'll find the source code and essential scripts for the project.

- **train.py**:
This script is the orchestrator of the training process. When activated, it calls upon environment.py to set the stage for RL algorithms. train.py initializes the environment, activates the specified RL agent according to the chosen algorithm (indicated by argument), and establishes a crucial connection between the environment and the RL agent. It plays a pivotal role in orchestrating the iterative learning process, defining the number of desired epochs through the provided argument.

- **environment.py**:
This foundational script shapes the environment for RL algorithm training. Its versatile nature enables it to expertly manage the run.py script, bridging the gap between the Gazebo simulation, the TurtleBot3's ball-striking actions, and the RL agent. It acts as the conduit through which essential data flows from run.py to the RL agent, providing the agent with the necessary information to optimize velocity (V) and angle (Theta). Furthermore, environment.py facilitates the transmission of these crucial V and Theta values from the RL agent to run.py, ensuring a seamless iterative process. Whether orchestrating the operations of run.py or independently performing vital calculations, environment.py lies at the heart of the training process. It essentially manages the environment, ensuring smooth interactions between the Gazebo simulation, the robot's actions, and the RL agent.

- **run.py**:
This script serves as an automated conductor for executing commands from the commands.txt file. Each command in commands.txt, demarcated by ---...---, operates in a distinct process. Results from commands commencing with #! are systematically collected through pipes. Importantly, run.py effectively coordinates the execution of the AIR2.py script within the Gazebo simulation environment. This automation ensures that the simulation seamlessly progresses from one iteration to the next without necessitating manual intervention to restart the simulation after each cycle.

- **AIR2.py**:
A pivotal robot script responsible for receiving information about its position, target goal, velocity, and angle to strike the ball with precision. This script not only calculates the final location of the ball, facilitating distance measurements from the target goal, but also orchestrates the Gazebo simulation, controlling the robot to perform the ball strike.

## Getting Started

Welcome to our project! To get started, follow these simple steps:

1. **Clone the Repository**

   - Begin by cloning this repository to your local machine.

2. **Code and Scripts**

   - Navigate to the `/code` directory to access the project's source code and various scripts.

3. **Project Documentation**

   - For comprehensive project documentation, explore the `/docs` directory.

4. **Training the Models**

   - To train one of the models within our environment, run the `train.py` script. You can customize the training process using the following parameters:
      - Specify the number of epochs (default is 1000).
      - Choose the reinforcement learning algorithm you want to use during training. Options include `ddpg`, `ppo`, or `sac`, with `ddpg` as the default.
   
   *Note:* When selecting an algorithm, make sure to provide the number of epochs parameter as well.

5. **Visualization of Training Progress**

   - After training, the reward distribution is displayed as a graph, showing reward values across epochs.

6. **Manual Testing**

   - To manually test the models with a small number of samples, execute the `test.py` script. Use the same parameters as in `train.py` to specify the algorithm and the number of epochs for testing.

## Lab Report

- [Read the Lab Report](/docs/AIR2_project.pdf)

## License

This project is licensed under the [MIT License](https://opensource.org/license/mit/), allowing you to use the code and resources for your research and development.

## Contact

For inquiries or collaborations, feel free to contact us at [asafephraim@gmail.com] or [assaf.bamberger@gmail.com].

Happy experimenting!

Assaf Bamberger & Asaf Ephraim
