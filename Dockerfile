FROM ubuntu:20.04

ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Paris

RUN apt-get update && apt-get -y install git python2.7 flex bison gcc g++ \
    make pkg-config libeigen3-dev libpng-dev libjpeg-dev libspiro-dev \
    libfmt-dev wget texlive-latex-base texlive-latex-extra tzdata ca-certificates \
    texlive-fonts-recommended bash --no-install-recommends && rm -rf /var/lib/apt/lists/*

# Installing Cmake
ENV CMAKE_VERSION=3.23
ENV CMAKE_BUILD=2

RUN wget https://github.com/Kitware/CMake/releases/download/v$CMAKE_VERSION.$CMAKE_BUILD/cmake-$CMAKE_VERSION.$CMAKE_BUILD-linux-x86_64.sh -O cmake.sh \
    && chmod +x cmake.sh && mkdir -p /opt/cmake && ./cmake.sh --skip-license --prefix=/opt/cmake

ENV PATH="/opt/cmake/bin:${PATH}"

# Installing IBEX
RUN git clone -b actions https://github.com/lebarsfa/ibex-lib.git && cd ibex-lib \
    && mkdir build && cd build && cmake .. && make -j8 && make install

# Installing CODAC
RUN git clone https://github.com/Teusner/codac -b dev --single-branch && cd codac \
    && mkdir build && cd build && cmake .. && make -j8 && make install

# Installing IPE-GENERATOR
RUN git clone https://github.com/Teusner/ipe_generator -b dev --single-branch && cd ipe_generator \
    && mkdir build && cd build && cmake .. && make -j8 && make install

# WakeBoat
RUN mkdir -p /WakeBoat/build && mkdir /output
COPY src /WakeBoat/src
COPY test /WakeBoat/test
COPY extern /WakeBoat/extern
COPY CMakeLists.txt /WakeBoat

RUN cd /WakeBoat/build && cmake .. -DWITH_IPE=ON -DCMAKE_BUILD_TYPE=Release && make -j8

ENTRYPOINT ["/WakeBoat/build/main", "-p", "/output", "-d", "30", "-s", "0.1", "--precision", "0.1", "-v"]