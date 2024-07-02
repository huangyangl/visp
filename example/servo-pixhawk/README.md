# cmake 
cd ~/shd/visp-ws/visp-build
cmake ../visp -DCMAKE_BUILD_TYPE=debug -DUSE_BLAS/LAPACK=GSL
# make 
~/shd/visp-ws/visp-build
make -j6 servoPixhawkDroneIBVS-test
# run
cd ~/shd/visp-ws/visp-build/example/servo-pixhawk
./servoPixhawkDroneIBVS-test --help
./servoPixhawkDroneIBVS-test --tag-size 0.12 --co udp://:14550
# wsl sitl
wsl
cd ~/code/hyl-git-repo-ws/sitl-master
sim_vehicle.py -v ArduCopter --console --out "192.168.100.197:14550" --out "127.0.0.1:14550" -w
# RTSP server
cd ZLMediaKit/release/linux/Debug
./MediaServer -h
sudo ./MediaServer -d -l 0 &
sudo killall -2 MediaServer
# RTSP pull
ffplay -rtsp_transport tcp -i rtsp://127.0.0.1:554/live/0


# VSCode gdb
launch.json:
{
    // Use IntelliSense to learn about possible attributes.
    // Hover to view descriptions of existing attributes.
    // For more information, visit: https://go.microsoft.com/fwlink/?linkid=830387
    "version": "0.2.0",
    "configurations": [
        {
            "name": "(gdb) Launch",
            "type": "cppdbg",
            "request": "launch",
            "program": "/home/jetson/shd/visp-ws/visp-build/example/servo-pixhawk/servoPixhawkDroneIBVS-test",
            "args": ["--tag-size","0.12","--co","udp://:14550"],
            "stopAtEntry": false,
            "cwd": "${fileDirname}",
            "environment": [],
            "externalConsole": false,
            "MIMode": "gdb",
            "setupCommands": [
                {
                    "description": "Enable pretty-printing for gdb",
                    "text": "-enable-pretty-printing",
                    "ignoreFailures": true
                },
                {
                    "description": "Set Disassembly Flavor to Intel",
                    "text": "-gdb-set disassembly-flavor intel",
                    "ignoreFailures": true
                }
            ]
        },


    ]
}