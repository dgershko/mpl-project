FROM universalrobots/ursim_e-series:5.16.0

ARG URCAP_VERSION=1.0.5 # latest version as if writing this
RUN apt-get update && apt-get -yqq install curl && apt-get clean
RUN curl -L -o /urcaps/externalcontrol-${URCAP_VERSION}.jar \
  https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases/download/v${URCAP_VERSION}/externalcontrol-${URCAP_VERSION}.jar

CMD ["/bin/bash"]
