#!/bin/sh

# 启动 openocd 加载配置文件

openocd -s openocd_scripts -f renesas.cfg -d3
