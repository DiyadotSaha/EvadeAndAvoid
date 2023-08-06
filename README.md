## Evasive Maneuvers

## Description
In this project, we plan on creating an additional autopilot functionality to add to the flight simulator. This new feature, known as Evasive Maneuvers, will enable the aircraft to operate under new control laws, ensuring that it avoids any expanding explosive forces that may be encountered along its trajectory.
The following classes will be created or edited to implement this: 

`ece163.Controls.VehicleClosedLoopControl`<br>
`ece163.Modeling.Explosion` (*NEW)<br>
`ece163.Display.vehicleDisplay`<br>
`ece163.Display.baseInterface`<br>

The goal of our project is to enhance our already existing autopilot model by adding in new control laws that will avoid the bombardment of explosions. A gauge of our success will be having a high-level state machine that manipulates the control values in a way that will make the aircraft maneuver in a visually and computationally effective way. i.e the aircraft movements visually mimic that of maneuvers from the research: “Inward” , “Outward”, “Pull Up”, “Dive”¹ and its position is not within the explosion radius of the missile. Another tool that will be used to gauge our success is the addition of GUI code to display the explosion of the missiles and a planned flight path line of our airplane 

## Installation
To run EvadeNAvoid.py, PySide6 needs to be installed. You can install it from the terminal by using the command 
`pip install PySide6`.

## Running the program 
To showcase the functionality of our program, you should run the EvadeNAvoid.py script. Upon doing so, a Python window shall open up, presenting the simulation in action. By zooming out, one can see all the explosions occurring along the aircraft's intended path.

## Visuals
The following representation depicts the visual manifestation of an explosion that originates from the coordinate point of 0,0,0 within our simulation. This event is characterized by a maximum radius of 80 meters and is steadily increasing at a rate of 53.33 meters per second. 
![An instance of the explosion at 0,0,0 with a maximum radius of 80 meters](Explosion.gif)

The following representation portrays the visual representation of multiple explosions transpiring simultaneously. It is expected that at a given moment, three explosions will take place. As the aircraft advances, these explosions will transpire in closer proximity to the aircraft, confined within a specific range. 
![Multiple instances of bombs exploding](Multiple_explosion.gif)

## Contributes 
Robert Gaines [regaines@ucsc.edu] <br>
Kenny Kim &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[kemkim@ucsc.edu] <br>
Diya Saha &nbsp;&nbsp;&nbsp; &nbsp;&nbsp;   [dsaha4@ucsc.edu] <br><br>
Pull requests are welcome. For major changes, please open an issue first
to discuss what you would like to change.
Please make sure to update tests as appropriate.
## Status
We have successfully implemented the explosions on the graphical user interface (GUI) and established a highly efficient path for the aircraft to follow. Additionally, we have now refined the waypoint following algorithm, ensuring minimal deviation and ensuring smoother navigation along the intended trajectory. Moreover, we have incorporated a robust state machine that enables the aircraft to identify and move toward the next safe point effectively, mitigating any potential explosive hazards. 
