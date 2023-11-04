#!/bin/bash

podman run -it --network host --ipc=host --pid=host\
    --env BS_ENV=$(basename "$BS_ENV") \
    --env BS_PROFILE="collection" \
    --env CDC_LOCALHOST=xf28id1-srv2.nsls2.bnl.local \
    --env EPICS_CA_ADDR_LIST=10.66.219.255 \
    --env EPICS_CA_AUTO_ADDR_LIST=no \
    --mount type=bind,source=/nsls2/conda/envs,target=/opt/conda/envs,readonly \
    --mount type=bind,source=/nsls2/data/pdf/shared/config/bluesky/profile_collection,target=/root/.ipython/profile_collection,readonly \
    --mount type=bind,source=/nsls2/software/etc/tiled/profiles,target=/root/.config/tiled/profiles,readonly \
    --mount type=bind,source=/etc/bluesky,target=/etc/bluesky,readonly \
    ghcr.io/nsls2/erobs-bsui:latest