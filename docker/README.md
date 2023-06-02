## To run the contianers
```bash
cd docker/ursim
docker build -t ursim:latest .
cd ../ur-driver
docker build -t ur-driver:latest .
cd ../ur-example
docker build -t ur-example:latest .
docker-compose up
```