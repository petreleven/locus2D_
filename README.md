# locus2D
**locus2D** is a  physics engine made in C++ with **Raylib** as the rendering abstraction.


- **Collision Detection:** pending
- **Verlet Integration:** pending


### Features
- **Particle system:** A particle system with configurable rules on the generation speed, age of particles, as well as the damping ratio. Check out the firework simulations using this system

  <img src="./demo_videos/fireworks.gif" alt="Fireworks Demo" width="400" height="300">
- **Spring and harmonic damped spring system:** A configurable spring system that can be used to simulate behaviours such as a camera follow system, vehicle suspension etc. The demo illustrates a bridge made using the springs with the endpoints unaffected by gravity
  <img src="./demo_videos/spring_bridge.gif" alt="Fireworks Demo" width="400" height="300">

### Getting Started
**THIS PROJECT USES CMAKE AS THE BUILD TOOL**
1. **Clone the Repository:**

    ```bash
    git clone https://github.com/yourusername/locus2D_.git
    ```

2. **Build the Project:**

    Navigate to the project directory and follow the build instructions to compile the engine and run the examples.

    ```bash
    cd locus2D
    mkdir build
    cd build
    cmake ..
    make
    ```

