## Configuración

1. ### Creación de espacio de trabajo
```bash
cd ~
mkdir -p leonardo_ws/src
```
2. ### Descarga de repositorio
```bash
cd leonardo_ws/src
git clone https://github.com/morg1207/Leonardo-Bot.git
```
3. ### Descargar e instalar dependencias
```bash
cd ~/leonardo_ws
rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
```
4. ### Compilación de espacio de trabajo

```bash
cd ~/leonardo_ws
source /opt/ros/humble/setup.bash 
colcon build --symlink-install
source install/setup.bash
```
4. ### Ejecutar cámara

```bash
cd ~/leonardo_ws
source install/setup.bash
ros2 launch leonardo_web leonado_web.launch.py
```

