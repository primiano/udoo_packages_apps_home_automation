#!/bin/bash
adb connect udoo
vulcanize -o index.html html/index.html --inline && adb push index.html /data/home_automation/
