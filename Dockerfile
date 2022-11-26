FROM opendronemap/odm:latest

WORKDIR /
RUN apt-get update && \
    apt-get install -y git

ENTRYPOINT ["python3", "/code/run.py"]