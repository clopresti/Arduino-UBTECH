#!/bin/bash

rm ./Arduino-UBTECH.zip
zip -r Arduino-UBTECH.zip . -x "*.git*" -x "*.vscode*" -x "*.DS_Store*" -x "build.sh" -x "Arduino-UBTECH.ino"
