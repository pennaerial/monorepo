#!/bin/bash

ssh xxxxx
cd /.......

git pull origin main

gh run download --name my-built-artifact --dir ./dist --overwrite

## Use the built obj 
chmod +x ./dist/my-program
sudo systemctl restart my-app.service

echo "Deployment complete!"