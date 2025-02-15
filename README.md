# Welcome 
## Create Catkin Workspace and virtual environment
```
mkdir catkin_ws 
cd catkin_ws 
mkdir src 
cd src 
mkdir external_libraries
python -m venv .venv
source .venv/bin/activate
cd external_libraries
```

# Dependencies
Install these to the same python environment/instance that ros uses
## RCPELib
```
git clone https://github.com/rolizzlefoshizzle/RCPELib
cd RCPELib
cd stl_rom_library
git clone https://github.com/decyphir/STLRom
cd STLRom
pip install .
cd ../..
mkdir build
cd build
cmake ..
make
cd .. 
pip install .
cd ..
```

## STLFormulaProgression
```
git clone https://github.com/rolizzlefoshizzle/STLFormulaProgression
cd STLFormulaProgression/
mkdir build 
cd build 
cmake ..
make 
cd ..
pip install .
cd ..
```

## VP-STO
```
git clone -b jax-dev https://github.com/JuJankowski/vp-sto
cd vp-sto
pip install .
cd ..
```

## Others
```
pip install matplotlib numpy control empy catkin_pkg jax catkin_tools jaxlib rospkg
cd ..
```

# ros package
## Installation
In catkin_ws/src:
```
git clone https://github.com/rolizzlefoshizzle/roToGoROS
cd ..
catkin_make
```

**Important**: 
## Usage
- launch files ending wiht "Plot" visualize the sim (the ones without it are for benchmarking purposes)
- the launch files with 
    - "rob" use regular robustness with the original formula 
    - "rcpe" use rcpe to update the formula and allow for bounded memory 
    - "RoToGo" use formula progression to update the formula and allow for robustness-to-go
- Run with `roslaunch ro-to-go CASE.launch --screen`
- Configure package in config/params.yaml
    - most interesting formulas for "userInFormula" are stayIn2.stl and thinGap.stl

- **At the moment, just use rcpe and ro-to-go!** I think i haven't updated rob yet


