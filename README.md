![GitHub last commit](https://img.shields.io/github/last-commit/luis-cmenezes/ball-beam)
![GitHub repo size](https://img.shields.io/github/repo-size/luis-cmenezes/ball-beam)
![GitHub top language](https://img.shields.io/github/languages/top/luis-cmenezes/ball-beam)

# Ball-and-beam Project

<img src="https://encrypted-tbn0.gstatic.com/images?q=tbn:ANd9GcQwLzoRfZndQdFkY2GrG0Fa56MpodpUgbtSf7SZl1vbaQ&s" width="490" height="412">

> The project is part of the Digital Systems course, aiming at the control of any system.
> The chosen system is the one in the image, known as the ball-and-beam system, in which the objective is to control the position of the ball.
> Some other project details involve: design of classical controllers, application of digital control concepts, embedded ROS2.

## ⭐ Content
The project, currently under development, features:

- [x] Docker images for build e run (may change as needed)
- [x] Build and run scripts (may change as needed)
- [x] ROS potentiometer calibration package
- [x] Arduino/Matlab infrared distance sensor calibration package
- [x] ROS step test package (servo vs. table)
- [ ] Matlab simulation (servo vs. position) and controller development package
- [ ] ROS for in-run control

## ⚡ Electrical Diagram

The electrical diagram of the project includes an ESP32, for microros usage, a potentiometer used as a rotation sensor, an infrared sensor for distance measurement, and finally, a servo motor with its actuator power supply.

Connected as shown in the diagram below:

<img src="https://i.imgur.com/N8nYxcS.png" width="478" height="412">

## ⚙️ Build/Flash
This project has been developed to be independent of any local installations on the user's computer. Therefore, its only dependency not contained in this repository is [Docker](https://docs.docker.com/desktop/install/linux-install/). 
Please ensure its installation and has sudo privileges before using the build scripts.

In the utils/build_scripts folder, you will find scripts for building the project, which are:

- set-build-env.sh: Typically executed only once per computer, it sets up the build environment and flash for the microcontroller by creating and downloading Docker images.
- build-microros-msg.sh: Using the image provided by the microros creators, it builds custom messages defined specifically in the src/ball_beam_msgs folder and adds them to the library used by arduino-cli.
- build-esp-firmware.sh: Compiles and flashes any code (.ino) in the src/ESP_CODE_PKG package, which is configurable in the file itself, using arduino-cli.
- build-ros2-code.sh: Compiles ROS2 codes that run on the machine, such as potentiometer calibration, step test, and in-run control.
- full-build.sh: Executes the last three scripts in the correct order. If you're just using the project, this build is sufficient after setting up the environment.

It's worth mentioning that for these codes to function correctly, they should be executed from the utils/build_scripts folder. Additionally, some scripts may require sudo password for execution.

## 💻 Run
Assuming that the entire build process was successful, to execute the project, simply navigate to the utils/ folder and execute:

```./run-ball-beam.sh ```

This command will execute two terminals: One with the microros-agent, enabling the exchange of ROS messages through serial with the microcontroller, and another running inside a Docker container with ROS2 for executing local code commands (such as ros2 run or ros2 launch).

If you're only using the project, the following command executes the in-run control code:

```ros2 launch ...``` (UPDATE)

## 📖 References
["Ball & Beam: System Modeling", Control Tutorials for MATLAB&Simulink](https://ctms.engin.umich.edu/CTMS/index.php?example=BallBeam&section=SystemModeling)

## 🤝 Collaborators
- [Professor Éder Alves de Moura](http://www.feelt.ufu.br/pessoas/docentes/eder-alves-de-moura) 
- [Professor Pedro Augusto Queiroz de Assis](http://www.femec.ufu.br/pessoas/docentes/pedro-augusto-queiroz-de-assis)
- Student Breno Gonçalves
- Student João
- Student Lucas
- Student Dilermando
