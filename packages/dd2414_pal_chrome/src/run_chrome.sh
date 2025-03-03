#!/bin/bash

function copy_webapps()
{
    set -x
    opt_dir="/opt/pal-webapps"
    opt_webapps=`ls $opt_dir`
    local roshost=${HOSTNAME%?}c
    for wa in $opt_webapps;
    do
            echo $opt_dir/$wa/dist
        if [ -d $opt_dir/$wa/dist ]; then
            webapp_dir="$HOME/.pal/www/webapps/$wa"
            if [ ! -d $webapp_dir ]; then
                mkdir -p $webapp_dir
                echo "Copying $opt_dir/$wa/dist to $webapp_dir"
                cp -r $opt_dir/$wa/dist/* $webapp_dir

                sed -i "s@rosServerName:\"control\"@rosServerName:\"$roshost\"@g" $webapp_dir/scripts/*
            fi
        fi
    done
}


copy_webapps
DISPLAY=:0 XAUTHORITY=/home/pal/.Xauthority rosrun dd2414_pal_chrome start_chrome.py
