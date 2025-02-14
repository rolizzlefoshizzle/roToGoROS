#dependencies:
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
```

## VP-STO
```
git clone -b jax-dev https://github.com/JuJankowski/vp-sto
pip install .
```

## Others
```
pip install matplotlib numpy
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
- If using a specific python virtualenvironment: set the path in the `mac<blahBlah>.launch` files and use those
- Otherwise, use the `'linux<blahBlah>.launch` files
- launch files ending wiht "Plot" visualize the sim (the ones without it are for benchmarking purposes)
- the launch files with 
    - "rob" use regular robustness with the original formula 
    - "rcpe" use rcpe to update the formula and allow for bounded memory 
    - "RoToGo" use formula progression to update the formula and allow for robustness-to-go
- Run with `roslaunch ro-to-go CASE.launch --screen`
    - if it complains about a missing directory in 'bagFiles,' just make that directory rq
        - I'll turn logging off eventually
- Configure package in config/params.yaml
    - most interesting formulas for "userInFormula" are stayIn2.stl and thinGap.stl

- **At the moment, just use rcpe and ro-to-go!** I think i haven't updated rob yet


