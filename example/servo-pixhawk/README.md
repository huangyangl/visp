# cmake 
cd ~/shd/visp-ws/visp-build
cmake ../visp -DCMAKE_BUILD_TYPE=debug -DUSE_BLAS/LAPACK=GSL
cmake ../visp -DCMAKE_BUILD_TYPE=release -DUSE_BLAS/LAPACK=GSL
# make 
cd ~/shd/visp-ws/visp-build
make -j6 servoPixhawkDroneIBVS-test
# run servo-pixhawk in jetson
cd ~/shd/visp-ws/visp-build/example/servo-pixhawk
./servoPixhawkDroneIBVS-test --help
./servoPixhawkDroneIBVS-test --tag-size 0.12 --co udp://:14550 --distance-to-tag 2 --verbose --rtsp --cambottom
# run mavproxy in jetson
mavproxy.py --master=/dev/ttyACM0 --out=udpout:127.0.0.1:14550
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

# user guid
1. 叠加图像显示的开启和关闭
由宏定义 #define DISP_OverLayImg 决定。
2. 叠加图像的RTSP推流
开启：--rtsp
关闭：缺省--rtsp，默认关闭
如果开启RTSP，必须先运行MediaServer以开启RTSP服务器再运行程序servoPixhawkDroneIBVS-test，否则servoPixhawkDroneIBVS-test程序会运行报错。
默认推流地址(目前只支持在代码中修改)：rtsp://127.0.0.1:554/live/0
3. 二维码的尺寸
--tag-size 0.12
4. 飞机到二维码的期望距离
默认为1
--distance-to-tag 2
5. 与飞控的连接方式
--co udp://:14550
6. 运行程序过程中显示冗余打印信息
--verbose
7. 默认图像处理频率为30fps
目前只支持在代码中改动后重新编译。
8. 相机的安装位置
安转在飞机正下方，相机x轴与飞机机体y轴同向： --cambottom
安转在飞机正左方，相机x轴与飞机机体x轴同向： 缺省--cambottom，默认关闭
9. 开启、关闭视觉伺服、降落
开启、关闭视觉伺服：左击图像，或者在命令行输入 0关闭，1开启
关闭视觉伺服并降落：右击图像，或命令行输入7
9. 运行顺序
（1）在机载计算机上运行mavproxy：mavproxy.py --master=/dev/ttyACM0 --out=udpout:127.0.0.1:14550
（2）在机载计算机上运行 RTSP server：cd ZLMediaKit/release/linux/Debug & sudo ./MediaServer -d -l 0 &
（3）在机载计算机上运行servo-pixhawk app：./servoPixhawkDroneIBVS-test --tag-size 0.12 --co udp://:14550 --distance-to-tag 2 --verbose --rtsp --cambottom


