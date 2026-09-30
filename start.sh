#!/bin/bash

# Absolute path to this script. /home/user/bin/foo.sh
SCRIPT=$(readlink -f $0)
# Absolute path this script is in. /home/user/bin
SCRIPTPATH=`dirname $SCRIPT`
cd "$SCRIPTPATH"

# Segmentador de solo: ./start.sh [gseg|patchwork]  (padrao: gseg)
# A session.yml liga a saida do escolhido na entrada do terrain_modelling.
export GROUND_SEG="${1:-${GROUND_SEG:-gseg}}"
case "$GROUND_SEG" in
  gseg|patchwork) ;;
  *)
    echo "Uso: $0 [gseg|patchwork]   (recebido: '$GROUND_SEG')" >&2
    exit 1
    ;;
esac
echo "Segmentador de solo: $GROUND_SEG"

export TMUX_SESSION_NAME=simulation
export TMUX_SOCKET_NAME=DEM_Simulation

# start tmuxinator
tmuxinator start -p ./session.yml

# if we are not in tmux
if [ -z $TMUX ]; then

  # just attach to the session
  tmux -L $TMUX_SOCKET_NAME a -t $TMUX_SESSION_NAME

# if we are in tmux
else

  # switch to the newly-started session
  tmux detach-client -E "tmux -L $TMUX_SOCKET_NAME a -t $TMUX_SESSION_NAME" 

fi
