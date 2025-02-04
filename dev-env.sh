# docker run --mount type=bind,source=${HOME}/Documents/workspace/Documents/Verwaltung,target=/workspace -it --rm --platform linux/amd64 dev-env:beta bash 
# run this in the terminal to start the container
# afterwards attach VS Code to the container and open the trackinglib folder
docker run --mount type=bind,source=${HOME}/Documents/workspace/Documents/Verwaltung,target=/workspace -it --rm trackinglib:latest bash