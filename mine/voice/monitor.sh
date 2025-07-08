#!/bin/bash

DIR_TO_WATCH="/app/mine/voice"
FILE_TO_WATCH="output.WebM"

cd "$DIR_TO_WATCH" || exit

echo "监控中... (等待 $FILE_TO_WATCH 出现)"

while true; do
    if [[ -f "$FILE_TO_WATCH" ]]; then
        echo "检测到文件，开始语音识别..."
        python3 ./run.py && {
            echo "识别完成，安全删除文件"
            rm -f "$FILE_TO_WATCH"
        }
    fi
    sleep 1
done
