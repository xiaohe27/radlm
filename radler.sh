#!/bin/bash
SCRIPT_DIR=`dirname $(python -c "import os, sys; print(os.path.realpath(\"$0\"))")`

compile(){
	#echo PYTHONPATH="$SCRIPT_DIR/lib/parsimonious:$SCRIPT_DIR" python3.4 $SCRIPT_DIR/radler/main.py "$@"
	PYTHONPATH="$SCRIPT_DIR/lib/parsimonious:$SCRIPT_DIR" python3.4 $SCRIPT_DIR/radler/main.py "$@"
}

#ensure pervasives is compiled
perv_obj="$SCRIPT_DIR/radler/lib/pervasives.radlo"
if [ ! -f $perv_obj ]
then
	compile -c --roscpp_dest . ${perv_obj/.radlo/.radl} -o ${perv_obj}
	#echo "compiled pervasives.radlo"
fi

compile -O $perv_obj $@


