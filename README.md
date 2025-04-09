# lab_2025

# Лабораторная работа №2025

В данной лабораторной работе необходимо запустить проект, ознакомиться с его структурой и ключевыми файлами для выполнения индивидуального задания (Задание №2, вариант у преподавателя).

Проект доступен на GitHub: https://github.com/KulikovNA/lab_2025

## Ход работы

### Задание №1

Запустите проект, ознакомьтесь с его структурой и ключевыми файлами для выполнения индивидуального задания.

---

## Действия и результат

### Запуск без Docker

1. Скачивание репозитория с GitHub:

```bash
# Загрузка репозитория
git clone https://github.com/KulikovNA/lab_2025.git
```

2. Добавление предварительно обученных весов:

Скачайте с Яндекс.Диска предварительно обученные веса `yolo11n.pt` и поместите их по пути:
```
~/lab_2025/src/cv_basics/data/yolo11n.pt
```

Или же скачайте веса сразу с репозитория utlralytics и разместите их в нужном месте одной командой:

```bash
wget https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt -O ~/lab_2025/src/cv_basics/data/yolo11n.pt
```


3. Инициализация и сборка проекта:

```bash
cd ~/lab_2025
colcon build 
. install/setup.bash
```

4. Установка зависимостей:

```bash
sudo apt-get update && sudo apt-get upgrade -y && sudo apt-get install -y \
  ros-humble-graph-msgs \
  ros-humble-rviz-visual-tools \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  python3-pip \
  ros-humble-gazebo-* && \
pip3 install cv_bridge pyzmq zmq ultralytics

pip3 install 'numpy==1.26.4'
```

5. Запуск симуляции и модели:

```bash
ros2 launch two_wheeled_robot launch_sim_cv.launch.py
```

В RVIZ нажмите “Add” → добавьте топик `/video_with_predict/Image`.

---

### Запуск с Docker

1. Предварительно скачайте с Яндекс диска tar образ Dockers/docker_image_base/base_docker_lab2.tar. Загрузка и сборка Docker-образа:

```bash
docker load -i base_docker_lab2.tar
docker build -t base_docker_lab2 .
```

2. Скачайте репозиторий:

```bash
git clone https://github.com/KulikovNA/lab_2025.git
```

3. Добавьте веса:

Скачайте `yolo11n.pt` с Яндекс диска и поместите по пути:
```
~/lab_2025/src/cv_basics/data/yolo11n.pt
```
Или же скачайте веса сразу с репозитория utlralytics и разместите их в нужном месте одной командой:

```bash
wget https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt -O ~/lab_2025/src/cv_basics/data/yolo11n.pt
```

4. Сборка и запуск контейнера:

```bash
cd ~/lab_2025/docker
bash build_lab_2_docker.sh
bash run_docker_lab2.sh
```

После запуска — откройте RVIZ, нажмите “Add” и выведите топик `/video_with_predict/Image`.

---

## Результат

Рисунок 1 — результат работы приложения (окно RVIZ с предиктами ML-модели).

![Результат работы RVIZ](example/example_launch.png)

