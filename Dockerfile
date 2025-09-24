FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

# 安裝基本開發工具
RUN apt update && apt install -y \
    python3-colcon-common-extensions \
    python3-pip \
    build-essential \
    git \
    curl \
    vim \
    wget \
    sudo \
    lsb-release \
    gnupg2 \
    locales \
    && rm -rf /var/lib/apt/lists/*

# 安裝 OpenCV 與 RViz 相關依賴
RUN apt update && apt install -y \
    ros-humble-rviz2 \
    ros-humble-image-tools \
    libopencv-dev \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# 設定工作目錄與拷貝程式碼
WORKDIR /TDK_ros
COPY ./src ./src

# 預設指令為 bash
CMD ["bash"]
