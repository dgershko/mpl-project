FROM universalrobots/ursim_e-series:5.16.0


# RUN apt-get update && \
#     apt-get install -y software-properties-common && \
#     add-apt-repository ppa:openjdk-r/ppa && \
#     apt-get update && \
#     apt-get install -y openjdk-11-jdk
# RUN apt-get update && apt-get install -y openjdk-11-jdk  # For Debian-based images
# RUN yum install -y java-11-openjdk-devel  # Uncomment this line for Red Hat-based images
# COPY ./externalcontrol-1.0.5.jar /ursim/.urcaps/
COPY externalcontrol-1.0.5.urcap /urcaps/externalcontrol-1.0.5.jar
CMD ["/bin/bash"]
