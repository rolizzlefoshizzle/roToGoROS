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

## Dependencies
Install these to your new virtual environment. **Important**: for now clone these with ssh instead of https 
### RCPELib
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
**Note**: The dependency STLRom requires python > 3.10. if the venv is using an older version, you can go into their setup.py and change that to a lower value (hehe)

### STLFormulaProgression
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

### VP-STO
```
git clone -b jax-dev https://github.com/JuJankowski/vp-sto
cd vp-sto
pip install .
cd ..
```

### Others
```
pip install matplotlib numpy control empy catkin_pkg jax catkin_tools jaxlib rospkg
cd ..
```

## ros package
### Installation
In catkin_ws/src:
```
git clone https://github.com/rolizzlefoshizzle/roToGoROS
cd ..
catkin_make
```
### Usage
- launch files ending wiht "Plot" visualize the sim (the ones without it are for benchmarking purposes)
- the launch files with 
    - "rob" use regular robustness with the original formula 
    - "rcpe" use rcpe to update the formula and allow for bounded memory 
    - "rotogo" use formula progression to update the formula and allow for robustness-to-go
- Run with `roslaunch ro-to-go CASE.launch --screen`
- Configure package in config/params.yaml
    - most interesting formulas for "userInFormula" are stayIn2.stl and thinGap.stl
