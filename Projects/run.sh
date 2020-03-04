#!/bin/sh
APP=$1
if [ -d "$APP" ] ; then
cd $APP
make getlibs
make eclipse CY_IDE_PRJNAME=$APP
make program
cd ..
else
echo "No" $APP "folder in:"
pwd
fi

