w#!/bin/bash

# Navigate to the directory where the script is located
cd "$(dirname "$0")"

# Check if node_modules exists, if not install dependencies
if [ ! -d "node_modules" ]; then
    echo "Installing dependencies..."
    npm install
fi

# Open the default web browser to the app
# Use xdg-open for Linux, open for Mac, start for Windows
if which xdg-open > /dev/null; then
    xdg-open http://localhost:8080 &
elif which open > /dev/null; then
    open http://localhost:8080 &
elif which start > /dev/null; then
    start http://localhost:8080 &
else
    echo "Could not detect how to open browser. Please visit http://localhost:8080"
fi

# Start the application
echo "Starting Rocket Telemetry Dashboard..."
npm start
