#!/bin/bash

set -o errexit -o nounset -o pipefail

host="$1"
path="$2"
shift 2
rsync "$path" "$host":/tmp
filename=`basename "$path"`

_term () {
    kill -TERM "$pid" 2>/dev/null
}

trap _term SIGTERM

ssh "$host" -tt -C "/tmp/$filename $@" &
pid=$!
wait "$pid"
