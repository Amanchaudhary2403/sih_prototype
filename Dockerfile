# Use Python as the base image
FROM python:3.10-slim

# Install Node.js
RUN apt-get update && apt-get install -y curl \
    && curl -fsSL https://deb.nodesource.com/setup_18.x | bash - \
    && apt-get install -y nodejs \
    && apt-get clean

# Set working directory to the project root
WORKDIR /app

# Install Python dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy all project files into the container
COPY . .

# Build the React Frontend
WORKDIR /app/client
RUN npm install
RUN npm run build

# Install Node Server dependencies
WORKDIR /app/server
RUN npm install

# Expose the port the app runs on
EXPOSE 3001

# Start the Node server
CMD ["node", "server.js"]
