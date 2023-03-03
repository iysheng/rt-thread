#!/bin/sh

# 启动 openocd 加载配置文件

openocd -s openocd_scripts -s /home/red/just4github/openocd/install_local/share/openocd/scripts -f renesas.cfg -d3 -l /tmp/dbg_openocd_renesas.txt
