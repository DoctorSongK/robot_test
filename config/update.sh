#!/bin/bash
sudo chmod +x /usr/local/ale_bot/bak_agv_main
sudo rm -f /usr/local/ale_bot/agv_main
\cp -rf /usr/local/ale_bot/bak_agv_main /usr/local/ale_bot/agv_main
exit 0
