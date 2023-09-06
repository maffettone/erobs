#!/bin/bash
# This is a loose copy of bsui meant to be used as a minimal working example in a container.

. /opt/conda/etc/profile.d/conda.sh

# tack on additional PYTHONPATH information
if [ ! -z "$BS_PYTHONPATH" ]; then
    if [ ! -z "$PYTHONPATH" ]; then
        export PYTHONPATH=$PYTHONPATH:$BS_PYTHONPATH
    else
        export PYTHONPATH=$BS_PYTHONPATH
    fi
fi

# If the above has not defined BS_AN_ENV and BS_AN_PROFILE we will error out
# violently on `conda activate` below.
conda_cmd="conda activate $BS_ENV"
$conda_cmd || exit 1

# setup the command we will use to start IPython below
# Ignoring history in container because mounting from volume without permissions
ipython_cmd="ipython --profile=$BS_PROFILE --IPCompleter.use_jedi=False --HistoryManager.enabled=False"


if [[ "$BS_PROFILE_DIR" ]]; then
    ipython_cmd="$ipython_cmd --ipython-dir=$BS_PROFILE_DIR"
fi

$ipython_cmd "$@"