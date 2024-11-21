# NR-AI-MAC Installation

## Requirements

1. Install the ns-3 prerequisites listed in the following website

https://www.nsnam.org/wiki/Installation

2. Boost C++ libraries

```bash
sudo apt install libboost-all-dev
```

3. Protocol buffers

```bash
sudo apt install libprotobuf-dev protobuf-compiler
```

4. pybind11

```bash
sudo apt install pybind11-dev
```

5. RayRLlib


```bash
pip install -U "ray[data,train,tune,serve]"
```

## General setup 

Use theese commands to download and build the NR-AI-MAC module

1. Clone the repository

```bash
git clone https://github.com/navid-keshtiarast/ML-Framework-for-NR-U-MAC-Protocol-Design-Multi-agent.git
```

2. Configure and build the ns-3 library

```shell
./ns3 configure --enable-examples --disable-werror
./ns3 build ns3ai_nrmac_gym
```

3. Set up the python interfaces

```bash
pip install -e contrib/ai/python_utils
pip install -e contrib/ai/model/gym-interface/py
```


