## 🧠 Autonomous Agent Simulation

This project is a compact yet powerful simulation of an autonomous agent 
capable of perceiving, deciding, and adapting within a virtual environment. 

## 🎯 Purpose

To explore how autonomous agents can:
- Sense their environment
- Make decisions based on internal state and external stimuli
- Adapt behavior over time

This simulation is a sandbox for experimenting with cognition, control theory, 
and emergent behavior—ideal for educators, hobbyists, and researchers alike.

## 🧮 Agent Architecture

1. World Model: Knowledge about the environment and agent itself
2. Environment: A signal moving contineously and randomly with noise
3. Perceptor: A Kalman Filter which detects signal from noise
4. Inference Engine: A PID controller knows the goal and how to plan
5. Effector: Implement the tasks from the Inference Engine	

## 🧩 Features

- **Modular Agent Architecture**  
  Each agent has a perception module, decision logic, and action interface.

- **Stateful Simulation Loop**  
  Agents evolve over time, responding to changing conditions.

- **Minimal Dependencies**  
  Runs on standard Python with optional visualization via `matplotlib` or `pygame`.

- **Extensible Design**  
  Easy to plug in new sensors, goals, or learning mechanisms.

## 🚀 Getting Started

python agent.py

The agent starts working automatic
- the red dot is signal with noise
- the blue dot is filtered signal which agent believes true
- the green dot is its location after detection, process, inference, 
	planning and actions 

The agent stops when it finally catches the signal
- the green dot is close enough to the red dot
- a fire ball is the indicator of "I Catch You!"

Pressing SPACE key to show the history (trace) or not

Pressing ENTER key to restart the simulation	

📚 Philosophy
This project is more than code—it's a reflection on cognition, autonomy, 
and simulation. It draws inspiration from control theory, behavioral psychology, 
and decades of experiences in different industries.

## License

MIT License


