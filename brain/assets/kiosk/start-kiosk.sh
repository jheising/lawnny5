#!/bin/bash

cd ../www && python -m http.server 8000 & chromium-browser --kiosk http://lawnnyjr.local:8000