#FROM ubuntu:20.04
FROM nvidia/cuda:11.0.3-base-ubuntu20.04

WORKDIR /root

RUN apt-get update && apt-get upgrade -y && apt-get install -y software-properties-common

RUN	add-apt-repository ppa:deadsnakes/ppa && \
	apt-get install -y python3.7 && \
	apt-get install -y ffmpeg libsm6 libxext6 && \
	apt-get install -y python3-pip && \
	apt-get install -y python3.7-distutils && \
	apt-get install -y python3-apt

RUN python3.7 -m pip install --upgrade pip && python3.7 -m pip install --upgrade setuptools

RUN python3.7 -m pip install grpcio && python3.7 -m pip install protobuf

COPY requirements.txt .

RUN python3.7 -m pip install -r requirements.txt

RUN python3.7 -m pip install carla==0.9.15

RUN python3.7 -m pip install coloredlogs

RUN python3.7 -m pip install grpcio-tools

RUN python3.7 -m pip install torch torchvision torchaudio yolov5  --extra-index-url https://download.pytorch.org/whl/cu116

#RUN python3.7 -m pip install -qr https://raw.githubusercontent.com/ultralytics/yolov5/master/requirements.txt

RUN apt-get install -y libglfw3-dev

RUN python3.7 -m pip install open3d

RUN export DISPLAY=:0.0

COPY . .

EXPOSE 5555/tcp

# gRPC
EXPOSE 50051/tcp
EXPOSE 50052/tcp
EXPOSE 50053/tcp
EXPOSE 50101-50512/tcp

# Carla
EXPOSE 2000/tcp

RUN python3.7 -m grpc_tools.protoc -I./opencda/protos --python_out=. --grpc_python_out=. ./opencda//protos/ecloud.proto

ENTRYPOINT [ "python3.7", "vehiclesim.py" ]
