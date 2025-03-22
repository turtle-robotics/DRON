# DRON: Disaster Response Observation Network

The DRON Project is a tool designed to aid first responders in their response to structural fires by collecting and providing key data on the hotspots within the structure. The goal of the project is to create a swarm of drones equipped with lidar and infared cameras, capable of autonomously navigating a complex environment and reporting on the active development of the fire.

## Table of Contents

- [Project Overview](#project-overview)
- [Poster](#project-poster)
- [Team Members](#team-members)
- [Getting Started](#getting-started)
- [Contributing](#contributing)
- [License](#license)
<!--- 
- [Key Features](#key-features)
- [Hardware](#hardware)
-->

  
<!--
## Key Features

- **IMU Data Gathering**: Each drone works to self balance and steer using a dynamic control system hosted on each individual PI, which relies on the input data from the IMU sensor.

- **Customization**: The hand is personalized to Kaeden's specific needs, ensuring a comfortable fit and optimal functionality.

- **User-Friendly Interface**: The interface is designed to be easy to use and configure, making it accessible for Kaeden and their caregivers.

- **Continuous Improvement**: We are committed to ongoing development, incorporating feedback and advancements in EMG technology to enhance the hand's performance and capabilities.

## Hardware

- **Raspberry Pi Model 5**: The Pi acts as the host node for the on board ROS network, and handles the OpenCV image processing from the thermal and RGB cameras, as well as the IR and lidar interactions to generate usable data.
  - *RBG Camera*: The first camera that attaches to the PI is the RBG camera, mainly for testing and for potential manual control.
  - *Infared (IR) Camera*: The second camera that attaches to the PI is the IR camera, which is responsible for gathering the heat image for the
  - *Lidar Camera*: The lidar camera is responsible for gathering a point cloud for which the IR feed can be overlaid to generate a heat-point-cloud. 
  
- **Teensy Microcontroller (1/2)**: The first teensy serves as an interface to the IMU, and as the interface to the motor drivers. It works to bring only necessary data to the pi for processing, and to handle the brute of the PID-based dynamic control system work.
  - *IMU*: 
  - *Motor Driver(s)*: 

- **Teensy Microcontroller (2/2)**: The second teensy runs the communications, namely to the wifi node to communicate with other drones and to the final destination of the discovered data.
  - *Wifi module*: 

-->

## Project Overview

The Disaster Response Observation Network, more commonly known as DRON, is the student led project with the goal of aiding firefighters and first responders during large structural fires by providing the detailed information about the location, intensity and growth of fires is crucial to the proper response, and by improving the ability of first responders to actively respond to events, we intend to save lives. 

The primary objectives of this project are as follows:
- and implement a drone with all sensors for autonomous flight and data aquisition.
- Collaberate between drones for decision making and obstacle detection.
- Communicate live data to the first responders in an easy to understand format.

## Project Poster

![Poster-Fall-2024](resources/DRON-Poster-Fall-2024.png)

## Team Members

### Project Leadership
- **[Ian Wilhite](https://www.linkedin.com/in/ian-wilhite)**
  - Major: Robotics and Controls Engineering
  - Role: Project Lead
  - Year: Junior
 
- **[Elizabeth Hannsz](https://www.linkedin.com/in/elizabeth-hannsz-2932a51ba/)**
  - Major: Aerospace Engineering
  - Role: Mechanical Subteam Lead
  - Year: Senior

- **[Aidan Briggs](https://www.linkedin.com/in/aidan-briggs-108297250/)**
  - Major: Computer Science
  - Role: Software Subteam Lead
  - Year: Junior

- **[Jacob Fuerst](https://www.linkedin.com/in/jacob-fuerst)**
  - Major: Mechanical Engineering
  - Role: Electrical Subteam Lead
  - Year: Sophomore

### Mechanical Subteam

- **[Malcolm Ferguson](https://www.linkedin.com/in/malcolmkferguson)**
  - Major: Mechatronics Engineering
  - Role: Mechanical Team
  - Year: Sophomore
    
- **[Christus Creer](https://www.linkedin.com/in/fpchristuscreer)**
  - Major: Industrial Systems Engineering
  - Role: Mechanical Team
  - Year: Junior

- **[Alan Alvarado](https://www.linkedin.com/in/alan-alvarado-1797102ab)**
  - Major: Mechanical Engineering
  - Role: Mechanical Team
  - Year: Sophomore
 
- **[Felix Chim](https://www.linkedin.com/in/felix-chim-3066a6228)**
  - Major: Aerospace Engineering 
  - Role: Mechanical Team
  - Year: Junior
    
- **[Nahum Tadesse](https://www.linkedin.com/in/nahum-tadesse-51ba922b6/)**
  - Major: Mechanical Engineering
  - Role: Mechanical Team
  - Year: Sophomore

- **[Aaron Velez](https://www.linkedin.com/in/aaron-velez-1083bb2b4)**
  - Major: Manufacturing and Mechanical Technology
  - Role: Mechanical Team
  - Year: Junior

- **[Elizabeth Person](https://www.linkedin.com/in/elizabeth-person-041a51252/)**
  - Major: Aerospace Engineering
  - Role: Mechanical Team
  - Year: Freshman
    
### Software Subteam

- **[Alonso Peralta](https://www.linkedin.com/in/alonso-peralta-espinoza-715419212/)**
  - Major: Computer Engineering
  - Role: Software Team
  - Year: Sophomore
 
- **[Brian Russell](www.linkedin.com/in/brian-russell-275a361a3)**
  - Major: Aerospace Engineering
  - Role: Software Team
  - Year: Sophomore

- **[Treasa Francis](https://www.linkedin.com/in/treasafrancis/)**
  - Major: Computer Engineering
  - Role: Software Team
  - Year: Sophomore

- **[Nicholas Summa](https://www.linkedin.com/in/nicholas-summa-3905342b8/)**
  - Major: Computer Engineering
  - Role: Software Team
  - Year: Sophomore
 
- **[Jason Lev](https://www.linkedin.com/in/Jason-Lev-312492240/)**
  - Major: Computer Science
  - Role: Software Team
  - Year: Sophomore
 

### Electrical Subteam
 
- **[Jacob Adamson](https://www.linkedin.com/in/jacob-adamson/)**
  - Major: Electrical Engineering
  - Role: Electrical Team
  - Year: Senior
  
- **[Lucas Ybarra](https://www.linkedin.com/in/lucas-ybarra-847ba72b6/)**
  - Major: Electrical Engineering
  - Role: Electrial Team
  - Year: Sophomore
  
- **[Aneek Roy](https://www.linkedin.com/in/aneekroy/)**
  - Major: Electrical Engineering 
  - Role: Electrial Team
  - Year: Sophomore
 
- **[Quinn Belmar](https://www.linkedin.com/in/quinnbelmar/)**
  - Major: Mechanical Engineering 
  - Role: Electrial Team
  - Year: Sophomore

### Alumni

- **[Abel Ayala](https://www.linkedin.com/in/ug-abel-ayala-co2024/)**
  - Major: Multidiciplinary Engineering Technology
  - Role: Project Lead, Software & Mechanical


## Getting Started

To get started with DRON, follow these steps:

1. **Clone the Repository**:
   ```
   git clone https://github.com/yourusername/DRON.git
   ```

2. **Install Dependencies**: Depending on the software and hardware components used, install the required dependencies and libraries as outlined in the project documentation.

3. **Build the Drone**: Follow the instructions provided to construct the physical drone including all sensors. 

4. **Calibration and Testing**: Perform the necessary calibration and begin testing the drone (ideally in a controlled enviornment). *Please verify all [local FAA rules](https://www.faa.gov/uas/resources/community_engagement/no_drone_zone) and obtain [any necessary licenses](https://www.faa.gov/uas/commercial_operators/become_a_drone_pilot) before beginning operations.*

5. **Contribute**: If you have ideas or improvements to contribute, please refer to the [Contributing](#contributing) section below.

## Contributing 

We welcome contributions from the open-source community and anyone interested in enhancing this project. To contribute:

1. Fork the repository.

2. Create a new branch for your feature or bug fix:

   ```
   git checkout -b feature/new-feature
   ```

3. Make your changes and commit them:

   ```
   git commit -m "Add new feature"
   ```

4. Push your changes to your fork:

   ```
   git push origin feature/new-feature
   ```

5. Create a pull request to the `main` branch of this repository.

We appreciate any contributions, whether it's in the form of code, documentation, or suggestions.

## License

This project is licensed under the [MIT License](LICENSE), which means that you are free to use, modify, and distribute the code as long as you provide proper attribution and include the original license in your distribution. Please review the full license for more details.
