# syntax=docker/dockerfile:1
FROM brunneis/python:3.8.3-ubuntu-20.04
WORKDIR /app
RUN apt update -y
COPY . /app
RUN pip install -r requirements.txt
EXPOSE 5001
# CMD "python3 server.py"