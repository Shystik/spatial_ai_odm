FROM opendronemap/odm:latest

WORKDIR /
RUN apt-get update && \
    apt-get install -y git
RUN git clone https://github.com/OpenDroneMap/odm_data_aukerman.git datasets
RUN mkdir datasets/project/ && mv datasets/images datasets/project/

ENTRYPOINT ["python3", "/code/run.py"]