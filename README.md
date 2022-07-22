# MDO-HuRT-S
Multi-domain Operations Human-Robot Teaming Sandbox (MDO-HuRT-S). This repository is to document the autonomous robot setup for use in the MDO-HuRT-S.

### netcat to connect to "Front Seat" Nav Computer
#### TO READ
netcat <IP> <port>
for example: netcat 192.168.1.21 8002

#### To WRITE
  echo '$PSEAC,T,,20,-10,THR_ON*02' | netcat 192.168.1.21 8002

### SeaRobotics Surveyor M1.8 -- Autonomous Surface Vehicle (ASV)
![Surveyor M1.8](sr-surveyorm1-8-shore.jpg)
The bedrock of MDO-HuRT-S is the [SeaRobotics Surveyor M1.8](https://www.searobotics.com/products/autonomous-surface-vehicles/sr-surveyor-class) without any water sampling or lidar sensors.

![Network Setup](searobotics-setup.png)

#### Ubiquiti AC Bullet: Base Access Point
Connected to the router, this Ubiquiti AC Bullet is the Access Point to all the ASVs.

#### Ubiquiti AC Bullet: ASV
Each ASV has its own dedicated Ubiquiti AC Bullet WiFi radio attached in bridge mode.

#### BCU (navigation computer)

#### DAC (data acquisition computer

#### Center Camera
