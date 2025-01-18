# Docker manual

### Clone repository
```bash
git clone https://github.com/pgolikov/UDH2025_robotics
cd UDH2025_robotics
```

### Create an updated image (if Dockerfile have updates)

```bash
docker compose build
```

### Allow docker xhost for gui interfaces
``` bash
xhost +local:docker
```

### Create an instance of a container from image (in 'UDH2025_robotics' folder):
``` bash
docker run -it --privileged --ipc=host --net=host --user root \
--runtime=nvidia --gpus all \
--device /dev/dxg \
--device /dev/dri/card0 \
--device /dev/dri/renderD128 \
-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
-v ~/.Xauthority:/home/sim/.Xauthority \
-v ./:/home/sim/UDH2025_robotics:rw \
-e DISPLAY=$DISPLAY -e MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA \
-p 14570:14570/udp \
--name=px4 udh2025_robotics-drone_sim:latest bash
```


#### To enter the docker use:
``` bash
docker exec -it px4 bash
```

### In docker, run ONCE coping file and ONCE build px4 after instaltaion of doker.
``` bash
copy_files
cd ~/PX4-Autopilot
sudo make px4_sitl gz_x500
```

