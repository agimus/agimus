FROM gepetto/buildfarm:robotpkg-16.04

ENV QT_X11_NO_MITSHM=1

RUN apt update
RUN apt install -y bash-completion python-vcstool python-catkin-tools ipython
RUN apt install -y robotpkg-gerard-bauzil
RUN apt install -y robotpkg-py27-hpp-manipulation-corba robotpkg-talos-simulation robotpkg-py27-qt4-hpp-gepetto-viewer robotpkg-py27-qt4-hpp-gui ros-kinetic-vision-visp robotpkg-talos-metapkg-ros-control-sot

RUN mkdir -p /workspace && cd /workspace && catkin init && catkin config --install
COPY agimus.repos /workspace/src/

RUN echo -e "source /etc/bash_completion\nsource /workspace/install/setup.bash" >> /root/.bashrc
COPY .gitconfig /root/.gitconfig

WORKDIR /workspace
RUN vcs-import --input src/agimus.repos --recursive src
RUN catkin build
