## Installation
1) Download and install OpenModelica (https://openmodelica.org/download/download-windows/).
2) Compile model .mo into .fmu using OpenModelica.
3) Run 
g++ -std=c++17 -o IVP/Cosim/extract_fmu Cosim/extract_fmu.cpp
Cosim/extract_fmu "Model/Quadcopter_fmu.zip" "Model"

to unzip the fmu .zip file.

## Run in Cosim (internal solver)
1) Run
g++ -std=c++17 -o IVP/Cosim/step_fmu Cosim/step_fmu.cpp
IVP/Cosim/step_fmu

to step the model using the fmu's internal solver.

## Run in Model Exchange (external solver)
1) Run
g++ -std=c++17 -o IVP/ME/step_fmu ME/step_fmu.cpp
IVP/ME/step_fmu

to step the model using external euler forward solver.
