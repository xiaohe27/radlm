#!/bin/bash
SCRIPT_DIR=`dirname $(python -c "import os, sys; print(os.path.realpath(\"$0\"))")`
PYTHONPATH="$SCRIPT_DIR/lib/parsimonious:$SCRIPT_DIR" python3.4 $SCRIPT_DIR/radler/main.py $@
