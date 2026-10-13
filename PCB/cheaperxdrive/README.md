# ghostleg
Repository for the development of an acoustically and bio-mechanically accurate leg phantom.


reference schematics:
https://github.com/odriverobotics/ODriveHardware/blob/master/v3/v3.5docs/schematic_v3.5.pdf
https://github.com/makerbase-mks/ODrive-MKS/blob/main/Hardware/MKS%20ODrive_S%20schematic_v3.6.pdf
https://github.com/antonchromjak/MINI4/blob/master/MINI4.pdf

design requirements:
- single sided PCB to be fabricated with JLCPCB, try to keep cost within 35SGD per unit. single sided standard may be cheaper than economic.
- motor peak current is unlikely to exceed 30A, use mosfet JMSL0302AG. package and pinout of mosfets in schematic are compatible.
- all other components to be kept the same
- CANbus for communication