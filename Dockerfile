FROM gcc:12

RUN apt-get update && apt-get -y install python2.7 flex bison gcc g++ \
    make pkg-config libeigen3-dev libpng-dev libjpeg-dev libspiro-dev \
    apt-utils libfmt-dev wget texlive-latex-base texlive-latex-extra \
    texlive-fonts-recommended --no-install-recommends && rm -rf /var/lib/apt/lists/*

# Installing Cmake
ENV CMAKE_VERSION=3.22
ENV CMAKE_BUILD=2
RUN wget https://cmake.org/files/v$CMAKE_VERSION/cmake-$CMAKE_VERSION.$CMAKE_BUILD.tar.gz && tar -xzvf cmake-$CMAKE_VERSION.$CMAKE_BUILD.tar.gz && cd cmake-$CMAKE_VERSION.$CMAKE_BUILD/ \
    && ./bootstrap && make -j4 && make install

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
RUN git clone https://github.com/Teusner/WakeBoat && cd WakeBoat && git submodule init && git submodule update \
    && mkdir build && cd build && cmake .. -DWITH_IPE=ON && make -j8

RUN mkdir /output

ENTRYPOINT ["/WakeBoat/build/test/05-video", "-p", "/output"]