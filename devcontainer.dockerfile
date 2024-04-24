FROM incebellipipo/devcontainer:humble

# Copy python package dependencies
COPY requirements.txt /tmp/requirements.txt

# Install python package dependencies
RUN pip install -r /tmp/requirements.txt