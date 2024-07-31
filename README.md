# **locus2D**

**locus2D** is a C++ physics engine using **Raylib** for rendering abstraction.

## Features

- **Particle System**: 
  Configurable particle system with adjustable generation speed, particle age, and damping ratio. Check out the firework simulations:

  <img src="./demo_videos/fireworks.gif" alt="Fireworks Demo" width="250" height="250">

- **Spring and Harmonic Dampened Spring System**: 
  Configurable spring system for simulating behaviors such as camera follow, vehicle suspension, etc. Includes a bridge simulation with springs:

  <img src="./demo_videos/spring_bridge.gif" alt="Spring Bridge Demo" width="250" height="250">

- **Collision Detection**: Pending
- **Verlet Integration**: Pending

## Getting Started

**This project uses CMake as the build tool.**

1. **Clone the Repository:**

    ```bash
    git clone https://github.com/yourusername/locus2D.git
    ```

2. **Build the Project:**

    Navigate to the project directory and compile the engine:

    ```bash
    cd locus2D
    mkdir build
    cd build
    cmake ..
    make
    ```
