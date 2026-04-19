# 创建文件夹

首先打开终端，默认在家目录下
```bash
# 在这里默认你已经下载了ros2了并且配置好环境了

# wheeled_dog 可以自己随便起，ros2_ws和src不用修改
mkdir -p wheeled_dog/ros2_ws/src

cd ~/wheeled_dog/ros2_ws/src

# 使用ssh也行
git clone https://github.com/ice4133/wheeled_dog.git


# 此时文件路径就是~/wheeled_dog/ros2_ws/src/wheeled_dog
# 第二个wheeled_dog是包名
# 如果不想要这个可以修改

cd ../..

# 返回到ros2_ws目录下,wheeled_dog是包名
colcon build --packages-select wheeled_dog
source install/setup.bash
ros2 run wheeled_dog motor_controller_node

```


# （可选）修改包名方法
1. 修改package.xml中的name标签
2. 修改CMakeLists.txt中的project标签
3. 修改物理目录下的包名,以及include下的包名
4. 删去build、install、log

重新编译、source即可
