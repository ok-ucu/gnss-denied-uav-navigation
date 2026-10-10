FROM ubuntu:22.04
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y git sudo lsb-release tzdata python3-pip
RUN useradd -m -s /bin/bash dev && echo "dev ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
USER dev
WORKDIR /home/dev

# Клонування з повторами: GitHub інколи обриває з'єднання
RUN git config --global http.postBuffer 524288000 && \
    git config --global http.version HTTP/1.1 && \
    for i in 1 2 3 4 5; do git clone https://github.com/ArduPilot/ardupilot.git && break || (rm -rf ardupilot; sleep 5); done
WORKDIR /home/dev/ardupilot
RUN for i in 1 2 3 4 5; do git submodule update --init --recursive && break || sleep 5; done

RUN USER=dev Tools/environment_install/install-prereqs-ubuntu.sh -y
ENV PATH="/home/dev/ardupilot/Tools/autotest:/home/dev/.local/bin:${PATH}"
RUN ./waf configure --board sitl && ./waf copter

COPY --chown=dev:dev ext_nav.parm /home/dev/ext_nav.parm
COPY --chown=dev:dev send_vision.py /home/dev/send_vision.py

WORKDIR /home/dev/ardupilot/ArduCopter
CMD ["sim_vehicle.py", "-v", "ArduCopter", "--no-rebuild", "--add-param-file=/home/dev/ext_nav.parm", \
     "--out=udp:127.0.0.1:14550", "--out=udp:127.0.0.1:14551"]
