### Computational Fluid Dynamics Analysis ###

**Description:** Perform flow simulation on Ansys to analyze the flow performance through the aircraft.

**Tools:** SolidWorks, Ansys Fluent

**Setup""
Turbulance Model: Spalart Allmaras
Stable and robust for attached flow model 
Consider Re: 〖~4.68×10〗^5
Fuselage and wing: attached flow; tail wing, landing gear: locally separated flow 
Velocity assumed: 32 m/s
Cruising angle: 2°
Viscosity calculation: Sutherland Law (dependent on temperature in ideal gas condition)
Mesh statistics: 694691 cells
Boundary conditions:
Inlet: -32cos(2) along Z dir, -32sin(2) along Y dir
Outlet: -1.305cos(2) along Z dir, 1.305sin(2) along Y dir
<img width="1664" height="719" alt="image" src="https://github.com/user-attachments/assets/0eab10c0-53cc-455d-bff3-dacacaa23683" />


![CFD](https://github.com/Leilazehui/Leilazehui.github.io/blob/main/Assets/Vel-aircraft.png)

