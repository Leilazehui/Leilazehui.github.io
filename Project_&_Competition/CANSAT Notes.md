Notes: Leila

**Test**: next thursday - theories and equations, mission plan and technology 

**Venue**: MAR Building

**Lab**: Opposite street of MAR building


22 July 2026
---
Satellite construction, standards: types, advantages and disadvantages of sizes; cell phone component
---
**subsystem of the satellites:**

bus
- Elec. Power Supp
- Thermal
- Attitude
- OBC
- TT&X
- Propusion
- Structure

---
Payload
- Sci. Instr.
- Transponder
- Meterological Inst.
- Navigat
- Military
---

EPS:
- Solar generator
- Thermoelectric Radioisotope Thermoelectric Generator (RTC): to provide heat and energy when probes are far away from the Earth, e.g., Mars rovers

Attitude control:
- optical fibre for attitude calculation 

AOCS Actuators:
- Flying wheels
- Magnetic actuators
- Control momentum gyro
- reaction control thruster (CHEMICAL)

Propulsion: Cold vs Hot Gas
---
Hot gas:
- Mono propellant
- Bi propellant: Kerosene, mathane, hydrogen etc + Oxygen
- Solid propellant boosters: **study the N-t diagram of thrust of solid fuel of the rocket** (good side: enormous amount of thrust; down side: cannot be regulated)
- Ion propulsion: Xenon ion pushed by electron
- Solar sails: photons


Structures and mechanisms:
- deployment, folding spring mechanism for modules, e.g., solar panels, antenna

---
Contact to Ground station: Berlin and Hainan? OCI project?

**Ground station at poles:**
- Svalbard in north pole, san martin base in south pole
---
Cost of space launches:
- Source: Our world in data
- heavy payload - falcon series: 2000 dollar /kg
- medium to light payload - 2000-6000 dollar/kg
---
Consequences of space derbis: keslar syndrom? need to clean the space debris to make sure the launch arent hindered by them.

23 July 2026 Electrical engineering 
---

- Avalanche
- Read the code of resistor 
- SMD resistor: surface mounted device resistor
- study eletrcical theories, laws and equations: ohm's law, kirchoff's law,

Prepare powerpoint presentation (around 8 minutes) on next week:
- team management: introduce team members, name of the team, communication within team
- work breakdown structure (with responsibilities), work shcedule
- first concept ideas (cansat, parachute removal)
- requirements and objectives


---
test content:
Lecture 1:
- space enviornemnt
- testing methods
- overview of space economy (number of satellites)
- satellite orbits
Lecture 2:
- Satellite techonology
- and statellit lecture from 22 July

basic calcualtion for the elcetrical parts: calculate power and voltage etc, rockets, velocity (basically all equations)
principle of energy conversion (mech to electrical etc)
20 Q and 1 bonus
Jupyter Notebook

27 July 2026
Software: Microcontroller (I/O pins)

---

RISK: reduced instructions computer
Program Memory - Flash - EEPROM (non-volatile: remain its content after the computer is shut down) vs volatile: lost memory after it's shut down, but not this case)
Voltage range: 3.3-5V
SPI/ UART/ watch dog timer (to reset a loop and to avoid lagging/error happening. WDT will restart everything again)/ interrupts/ timers

**C language:**
#### e.g. FATFS* fs
FATFS = a type (a struct from the lib of FatFs
With the *, it means it's a pointer to FATFS
fs = name of the variable, holds the memory address where a FatFs obj is stored, but not the actual data

#### Stage: preprocessing, compiling and linking
1. "#" marked: it is a specific operation to recognize and execute special preprocessor instruction which are not implmented in C
2. transform higher level source code to object code (machinec ode, missing functions that cannot be translated)
3. Linking: find where other object files located and compile them together, transform missing functions

#### Binary system vs Hexadecimal system

#### Two's complement
8-bit vairables: express 256 conditions for positive values: 0 to +127, and positive to negative values: -128 to +127
to leftmost number indicates whether the value is pos or neg. 0 = neg, 1 = pos
unsigned value is only for positive value ranging from 0 to +255

#### Adruino:
- 1KB Electrical programmable memory: read once, progress for thousands of times, can be erased or reprogrammed by UV lights
- Atmel Atmega328P microcontroller
- Flash memory: volatile memory
- 14 digital I/O
- siz 10bit ADC channels

- EEPROMS takes longer time and more energy than Flash memory, so good for configuration parameters, but not the fast processing program
- ADC: 2 to the 10 steps, together 1024 steps (the analog-to-digital converter)

- Aruidno data type definitions:
- Boolean - for C++ but not C; can be true/false, just need one bit of memory, e.g., boolean absolute_truth = true;
- Byte - 8 bits, obtain values from 0 to 255, e.g., byte my_variable = 0;
- Char - 1 byte sign of the ASCII code written with an aprostrophe, e.g., char 'A', that means it's 65
- Unsigned char - only contain unsigned character, which is pos value from 0 to 255
- unsigned char my_char = 171, that means the variable saves the number 171
- Int interger - constss of two bytes with values from -32768 to +32767
- Unsigned int - 0 to 63535 (2^16 - 1)
- Unsigned long - 32bit variable with pos values from 0 to 4 294 967 295 (very large variable)
- Float - floating point vairable which can store 32bit including algebraic sign and exponent. they range from -3.4028235 E+38 to +3.4028235 E+38
- String - array of characters including a null-terminated string, e.g., string my_sign[] = "Hello World", **this is a 12 byte string, 11 cahracters with 1 more position for "0" to let the computer know it stops at the null terminator**

#### Arduino operator
| bit by bit OR;
~ bit by bit NOT;
|| OR, e..g, A||B true, if A or B not zero;
! A!B false;

#### Control structure
while - loop continuously and infinitely, until the expression inside the parenthesis () becomes false
do...while - 
for - 

SMD edition:
Mega16U2 is the programer, which program mega328P to run the program

28 July 2026 (CAD design, Msc Paul Thurley)

--- 
Regulations and restriction: ECSS
MBSE: EGSE, tools, DESC, models etc
Satellite structure optimization: quazi static load - **most important!!**

