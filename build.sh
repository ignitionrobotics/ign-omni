#!/bin/bash

set -e

SCRIPT_DIR=$(dirname ${BASH_SOURCE})

"$SCRIPT_DIR/tools/packman/python.sh" "$SCRIPT_DIR/tools/repoman/repoman.py" build "$@" || exit $?
