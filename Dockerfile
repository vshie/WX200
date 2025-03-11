FROM python:3.11-slim-bullseye

# Install minimal system dependencies
RUN apt-get update && apt-get install -y \
    gcc \
    python3-dev \
    psmisc \
    libxml2-dev \
    libxslt-dev \
    && rm -rf /var/lib/apt/lists/*

# Upgrade pip first and disable hash checking
RUN pip install --upgrade pip

# Install Python dependencies without hash verification
RUN pip install --no-cache-dir --no-deps flask==2.3.3 && \
    pip install --no-cache-dir --no-deps flask-cors==4.0.0 && \
    pip install --no-cache-dir --no-deps pyserial==3.5 && \
    pip install --no-cache-dir --no-deps requests==2.31.0 && \
    pip install --no-cache-dir --trusted-host pypi.org --trusted-host files.pythonhosted.org pymavlink==2.4.40 && \
    pip install --no-cache-dir --no-deps Werkzeug==2.3.7 && \
    pip install --no-cache-dir --no-deps Jinja2==3.1.2 && \
    pip install --no-cache-dir --no-deps MarkupSafe==2.1.3 && \
    pip install --no-cache-dir --no-deps itsdangerous==2.1.2

# Create app directory
WORKDIR /app

# Create logs directory inside the container
RUN mkdir -p /app/logs && chmod 777 /app/logs

# Copy app files
COPY . /app

# Set environment variables
ENV PYTHONUNBUFFERED=1
ENV FLASK_APP=main.py

# Expose port for Flask
EXPOSE 6567

LABEL version="0.1"

ARG IMAGE_NAME
LABEL permissions='\
{\
  "ExposedPorts": {\
    "6567/tcp": {}\
  },\
  "HostConfig": {\
    "Binds": [\
      "/usr/blueos/extensions/wx200:/app/logs",\
      "/dev:/dev",\
      "/dev/serial/by-id:/dev/serial/by-id",\
      "/dev/serial/by-path:/dev/serial/by-path",\
      "/dev/ttyUSB0:/dev/ttyUSB0"\
    ],\
    "ExtraHosts": ["host.docker.internal:host-gateway"],\
    "PortBindings": {\
      "6567/tcp": [\
        {\
          "HostPort": "6567"\
        }\
      ]\
    },\
    "NetworkMode": "host",\
    "Privileged": true\
  }\
}'

ARG AUTHOR
ARG AUTHOR_EMAIL
LABEL authors='[\
    {\
        "name": "$Tony White + Claude",\
        "email": "$tony@bluerobotics.com"\
    }\
]'

ARG MAINTAINER
ARG MAINTAINER_EMAIL
LABEL company='\
{\
        "about": "",\
        "name": "$Tony White",\
        "email": "$tony@bluerobotics.com"\
    }'
LABEL type="tool"

ARG REPO
ARG OWNER
LABEL readme='https://raw.githubusercontent.com/$OWNER/$REPO/{tag}/README.md'
LABEL links='\
{\
        "source": "https://github.com/$OWNER/$REPO"\
    }'
LABEL requirements="core >= 1.1"

ENTRYPOINT ["python", "-u", "/app/main.py"]
