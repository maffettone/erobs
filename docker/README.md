## To run the contianers
```bash
cd docker/ursim
docker build -t ursim:latest .
cd ../ur-driver
docker build -t ur-driver:latest .
cd ../ur-example
docker build -t ur-example:latest .
cd ../
docker-compose up
```

Then the robot sim at `http://localhost:6080/vnc.html` needs to be turned on. 
In the installation tab, change the external control URCap IP to `192.168.56.102`. 
Then create a program for external control and start it. 
You may have to restart the ur-driver container. 