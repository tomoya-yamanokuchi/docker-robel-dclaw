# rm:         コンテナ終了時に自動的にコンテナを削除
# it:         -i + -t: 標準入力とTerminalをAttachする
# gpus:       all, または 0, 1, 2
# privileged: ホストと同じレベルでのアクセス許可
# net=host:   ホストとネットワーク名前空間を共有
# ipc=host:   ホストとメモリ共有

docker run --rm -it --gpus all --privileged --net=host --ipc=host \
--device=/dev/video0:/dev/video0 \
--device=/dev/video1:/dev/video1 \
--device=/dev/video2:/dev/video2 \
--device=/dev/video3:/dev/video3 \
-e DOCKER_USER_NAME=$(id -un) \
-e DOCKER_USER_ID=$(id -u) \
-e DOCKER_USER_GROUP_NAME=$(id -gn) \
-e DOCKER_USER_GROUP_ID=$(id -g) \
-v $HOME/.Xauthority:/home/$(id -un)/.Xauthority -e XAUTHORITY=/home/$(id -un)/.Xauthority \
-v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=unix$DISPLAY \
-v /dev/snd:/dev/snd -e AUDIODEV="hw:Device, 0" \
-v /home/$USER/workspace:/home/$USER/workspace \
-v /home/$USER/docker-robel-dclaw/sample_code:/home/$USER/sample_code \
-v /hdd_mount/ROBEL_DClaw_hands_on:/home/$USER/workspace/ROBEL_DClaw_hands_on \
docker_robel bash
