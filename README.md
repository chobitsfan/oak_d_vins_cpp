An utility processing features tracking data from a Luxonis OAK-D camera and sending them to a modified version of VINS-Fusion

## 1. Prerequisites
### 1.1. **depthai-core**
Tested with 2.25.0
```
    git clone https://github.com/luxonis/depthai-core
    git submodule update --init --recursive
    cmake -S. -Bbuild
    cmake --build build --target install
```

### 1.2. **OpenCV**
Tested with 4.6
```
    sudo apt install libopencv-dev
```

## 2. Build
```
    cmake -D'depthai_DIR=../depthai-core/build/install/lib/cmake/depthai' .
    make
```

## 3. Run
```
    ./feature_tracker
```
